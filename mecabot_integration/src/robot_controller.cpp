#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller"), manual_drive_active_(false), last_node_("None")
    {
        // --- SUBSCRIPTIONS ---
        // Vangt commando's op vanuit het adminpaneel of centrale systeem
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_command", rclcpp::QoS(10).reliable(),
            std::bind(&RobotController::handle_robot_command, this, std::placeholders::_1));
            
        // Luistert naar de pijltoetsen (GUI besturing) die binnenkomen
        mux_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/gui_cmd_vel", rclcpp::QoS(1).reliable(),
            std::bind(&RobotController::cmd_vel_callback, this, std::placeholders::_1));

        // --- PUBLISHERS ---
        // Stuurt de handmatige snelheden door naar het GUI-prioriteitstopic van jullie Drive_Mux
        gui_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/gui_cmd_vel", rclcpp::QoS(1).reliable());
        
        // Overige status- en coördinaten-publishers
        bt_node_pub_ = this->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", rclcpp::QoS(1).reliable().transient_local());
        bt_coord_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/btDriveCoord", rclcpp::QoS(10).reliable());
        dock_pub_ = this->create_publisher<std_msgs::msg::String>("/DockingCommand", rclcpp::QoS(10).reliable());
        manual_pub_ = this->create_publisher<std_msgs::msg::Bool>("/manual_drive_active", rclcpp::QoS(1).reliable().transient_local());

        // Initiële status verzenden
        publish_manual_flag();
      
        std_msgs::msg::String bt_msg;
        bt_msg.data = last_node_;
        bt_node_pub_->publish(bt_msg); 

        RCLCPP_INFO(this->get_logger(), "RobotController succesvol gekoppeld aan de Drive_Mux architectuur.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mux_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gui_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dock_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_node_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr bt_coord_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_pub_;

    bool manual_drive_active_;
    std::string last_node_;

    void handle_robot_command(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string cmd = msg->data;

        // "SHOW_LOST_SCREEN" mag de rij-statussen en actieve navigatie niet onderbreken
        if (cmd == "SHOW_LOST_SCREEN") {
            last_node_ = "None";
            std_msgs::msg::String screen_msg;
            screen_msg.data = "robot-error-drive";
            bt_node_pub_->publish(screen_msg);
            return;
        }

        // DRY-optimalisatie: Zet de statussen en stop lopende handmatige commando's
        manual_drive_active_ = (cmd == "MANUAL_DRIVE");
        publish_manual_flag();
        stop_robot(); 

        if (cmd == "STOP" || cmd == "MANUAL_DRIVE") {
            last_node_ = "None";
            if (cmd == "MANUAL_DRIVE") {
                RCLCPP_INFO(this->get_logger(), "Manuele besturing (pijltoetsen adminpaneel) actief.");
            }
        }
        else if (cmd == "START_QUIZ")           start_quiz_mode();
        else if (cmd == "DRIVE_QUIZ_LOCATION_A") drive_to_quiz_location_a();
        else if (cmd == "DRIVE_QUIZ_LOCATION_B") drive_to_quiz_location_b();
        else if (cmd == "DRIVE_TO_WORK_AREA")    drive_to_work_area_a();
        else if (cmd == "DRIVE_TO_CHARGING_AREA") drive_to_charging_area();
        else if (cmd == "DOCK")                  start_docking();
        else {
            RCLCPP_WARN(this->get_logger(), "Onbekend robot_command: %s", cmd.c_str());
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // MUX controle: Alleen doorsturen naar de multiplexer als de MANUAL_DRIVE-modus aan staat
        if (manual_drive_active_) {
            gui_cmd_vel_pub_->publish(*msg);
        }
    }

    void publish_manual_flag() {
        std_msgs::msg::Bool b;
        b.data = manual_drive_active_;
        manual_pub_->publish(b);
    }

    void stop_robot() {
        // Stuur een lege Twist (0-snelheid) naar het GUI-topic. 
        // Jullie eigen drive_mux lost de verdere afhandeling/timeout op op basis van prioriteit!
        geometry_msgs::msg::Twist t;
        gui_cmd_vel_pub_->publish(t);

        if (last_node_ == "robot-go-charge") {
            std_msgs::msg::String dock_msg;
            dock_msg.data = "STOP";
            dock_pub_->publish(dock_msg);
        }
    }

    void start_quiz_mode() {
        last_node_ = "robot-arrived-at-quiz-location";
        send_bt_node_signal(last_node_);
    }

    // Helper om posities te versturen zonder dubbele code
    void publish_pose(double x, double y, double z_rot, double w_rot) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.z = z_rot;
        pose.pose.orientation.w = w_rot;
        bt_coord_pub_->publish(pose);
    }

    void send_bt_node_signal(const std::string& node_name) {
        std_msgs::msg::String node_msg;
        node_msg.data = node_name;
        bt_node_pub_->publish(node_msg);
    }

    void drive_to_quiz_location_a() {
        last_node_ = "follow-robot-screen"; 
        send_bt_node_signal(last_node_);
        publish_pose(-1.3658989667892456, -1.4719496965408325, 0.371473234987957, 0.9284436631737987);
    }

    void drive_to_quiz_location_b() {
        last_node_ = "follow-robot-screen"; 
        send_bt_node_signal(last_node_);
        publish_pose(-4.064824104309082, 1.6677558422088623, 0.03214270147272485, 0.9994832898763417);
    }

    void drive_to_work_area_a() {
        last_node_ = "robot-startup";
        send_bt_node_signal(last_node_);
        publish_pose(-1.3658989667892456, -1.4719496965408325, 0.371473234987957, 0.9284436631737987);
    }

    void drive_to_charging_area() {
        last_node_ = "robot-go-charge";
        send_bt_node_signal(last_node_);
        publish_pose(0.0, 0.0, 0.0, 1.0); // Stuurt robot naar de oorsprong (laadstation)
    }

    void start_docking() {
        last_node_ = "robot-docking";
        send_bt_node_signal(last_node_);

        std_msgs::msg::String d;
        d.data = "START";
        dock_pub_->publish(d);
        RCLCPP_INFO(this->get_logger(), "Fysieke docking-procedure geactiveerd.");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
