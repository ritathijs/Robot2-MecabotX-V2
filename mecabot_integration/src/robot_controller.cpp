#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RobotController : public rclcpp::Node
{
public:
    RobotController() : Node("robot_controller")
    {
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_command", 10,
            std::bind(&RobotController::handle_robot_command, this, std::placeholders::_1));

        mux_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/robot_cmd_vel", 10,
            std::bind(&RobotController::Cmd_Vel_Callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        bt_node_pub_ = this->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        bt_coord_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/btDriveCoord", 10);
        dock_pub_ = this->create_publisher<std_msgs::msg::String>("/DockingCommand", 10);
        manual_pub_ = this->create_publisher<std_msgs::msg::Bool>("/manual_drive_active", 10);

        manual_drive_active_ = false;
        last_node_ = "None";

        RCLCPP_INFO(this->get_logger(), "RobotController gestart.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mux_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dock_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_node_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr bt_coord_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_pub_;

    bool manual_drive_active_;
    std::string last_node_;

    void handle_robot_command(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string cmd = msg->data;

        if(cmd == "STOP") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            last_node_ = "None";
        }
        else if(cmd == "MANUAL_DRIVE") {
            manual_drive_active_ = true;
            publish_manual_flag();
            stop_robot();
            last_node_ = "None";
            RCLCPP_INFO(this->get_logger(), "Manuele besturing actief.");
        }
        else if(cmd == "START_QUIZ") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            start_quiz_mode();
        }
        else if(cmd == "DRIVE_QUIZ_LOCATION_A") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            drive_to_quiz_location_a();
        }
        else if(cmd == "DRIVE_QUIZ_LOCATION_B") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            drive_to_quiz_location_b();
        }
        else if(cmd == "DRIVE_TO_WORK_AREA") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            drive_to_work_area_a();
        }
        else if(cmd == "DRIVE_TO_CHARGING_AREA") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            drive_to_charging_area();
        }
        else if(cmd == "DOCK") {
            manual_drive_active_ = false;
            publish_manual_flag();
            stop_robot();
            start_docking();
        }
        else if(cmd == "SHOW_LOST_SCREEN") {
            last_node_ = "None";
            std_msgs::msg::String screen_msg;
            screen_msg.data = "robot-error-drive";
            bt_node_pub_->publish(screen_msg);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Onbekend robot_command: %s", cmd.c_str());
        }
    }

    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_pub_->publish(*msg);
    }

    void publish_manual_flag() {
        std_msgs::msg::Bool b;
        b.data = manual_drive_active_;
        manual_pub_->publish(b);
    }

    void stop_robot() {
        geometry_msgs::msg::Twist t;
        cmd_vel_pub_->publish(t);
        if(last_node_ == "robot-go-charge") {
            std_msgs::msg::String dock_msg;
            dock_msg.data = "STOP";
            dock_pub_->publish(dock_msg);
        }
    }

    void start_quiz_mode() {
        // GECORRIGEERD: Stuurt nu direct de juiste string die de Pi-listener begrijpt!
        last_node_ = "robot-arrived-at-quiz-location";
        std_msgs::msg::String node_msg;
        node_msg.data = last_node_;
        bt_node_pub_->publish(node_msg);
    }

    void drive_to_quiz_location_a() {
        last_node_ = "follow-robot-screen"; 
        std_msgs::msg::String node_msg;
        node_msg.data = last_node_;
        bt_node_pub_->publish(node_msg);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = -1.3658989667892456;
        pose.pose.position.y = -1.4719496965408325;
        pose.pose.orientation.z = 0.371473234987957;
        pose.pose.orientation.w = 0.9284436631737987;
        bt_coord_pub_->publish(pose);
    }

    void drive_to_quiz_location_b() {
        last_node_ = "follow-robot-screen"; 
        std_msgs::msg::String node_msg;
        node_msg.data = last_node_;
        bt_node_pub_->publish(node_msg);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = -4.064824104309082;
        pose.pose.position.y = 1.6677558422088623;
        pose.pose.orientation.z = 0.03214270147272485;
        pose.pose.orientation.w = 0.9994832898763417;
        bt_coord_pub_->publish(pose);
    }

    void drive_to_work_area_a() {
        last_node_ = "robot-startup";
        std_msgs::msg::String node_msg;
        node_msg.data = last_node_;
        bt_node_pub_->publish(node_msg);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = -1.3658989667892456;
        pose.pose.position.y = -1.4719496965408325;
        pose.pose.orientation.z = 0.371473234987957;
        pose.pose.orientation.w = 0.9284436631737987;
        bt_coord_pub_->publish(pose);
    }

    void drive_to_charging_area() {
        last_node_ = "robot-go-charge";
        std_msgs::msg::String node_msg;
        node_msg.data = last_node_;
        bt_node_pub_->publish(node_msg);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = 0.0; 
        pose.pose.position.y = 0.0;
        pose.pose.orientation.w = 1.0;
        bt_coord_pub_->publish(pose);
    }

    void start_docking() {
        last_node_ = "robot-docking";
        std_msgs::msg::String d;
        d.data = "START";
        dock_pub_->publish(d);

        std_msgs::msg::String node_msg;
        node_msg.data = last_node_;
        bt_node_pub_->publish(node_msg);
        RCLCPP_INFO(this->get_logger(), "Fysieke docking gestart.");
    }
}; // <-- HIER hoort de klasse pas te sluiten met een puntkomma!

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}

