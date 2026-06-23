#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <std_msgs/msg/float32.hpp> 
#include <std_msgs/msg/bool.hpp>  
#include <geometry_msgs/msg/twist.hpp>
#include <fstream>
#include <vector>


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
using namespace std::chrono_literals;



// Decorator atijd-SUCCES
class ForceSuccess : public BT::DecoratorNode
{
public:
    ForceSuccess(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        // op moment decorator getikt wordt : hij moet zijn enig kind ticken
        const BT::NodeStatus child_state = child_node_->executeTick();

        // als kind RUNNING geeft
        if (child_state == BT::NodeStatus::RUNNING) {
            return BT::NodeStatus::RUNNING;
        }
        // ALS KIND GEEN RUNNING GEEFT, HETZIJ SUCCESS HETZIJ FAILURE
        return BT::NodeStatus::SUCCESS;
    }
};

// RECHTERKIND VAN FALLBACK OP MOMENT DAT ROBOT EN QUIZPI CONNECTIE VERLOREN ZIJN
class ConnectionLost : public BT::StatefulActionNode
{
public:
    ConnectionLost(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btConnectionLost");

        // Publisher naar BehaviorTreeNode
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

        // QoS en subscription naar /connection
        // Dit om berichtaankomst te garanderen
        // /connection wordt aangestuurd door quiz_bt_node()
        // Indien die code herverbinding opmerkt, stuurt deze CONNECT
        rclcpp::QoS qos(1);
        qos.reliable();

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/connection", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                last_connection_msg_ = msg->data;
            });


    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        // Publiceer zijn naam op BehaviorTreeNode
        std_msgs::msg::String msg;
        msg.data = name();
        pub_->publish(msg);

        last_connection_msg_ = "";
        std::cout << "[ConnectionLost] START - waiting for CONNECT" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        // als quit_bt_node heeft laten weten dat er een reconnect is gebeurd
        if (last_connection_msg_ == "CONNECT")
        {
            std::cout << "[ConnectionLost] CONNECT received -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[ConnectionLost] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::string last_connection_msg_;
};

// Node die altijd draait (bovenaan in BT) tenzij hij faalt
// Kijkt of quiz_bt_node laat weten dat er een disconnect heeft plaatsgevonden
class CheckNetworkError : public BT::StatefulActionNode
{
public:
    CheckNetworkError(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btCheckNetworkError");

        // reliable communicatie
        rclcpp::QoS qos(1);
        qos.reliable();

        // Subscriber naar /connection topic
        // topic van quiz_bt_node die info geeft over connect en disconnect
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/connection", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                last_connection_msg_ = msg->data;
            });
    }

    // Indien disconnect plaatsvind op moment dat robot in laadstation zit : geen probleem
    // Om dit te weten via blackboard status ophalen of robot aan het laden is
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<bool>("connection_chargeStatus") };
    }

    BT::NodeStatus onStart() override
    {
        last_connection_msg_ = "";
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_); // Kijken of er nieuwe berichten zijn op topic

        // Lees van het blackboard (default = false)
        bool charge_connected = false;
        getInput("connection_chargeStatus", charge_connected);

        // Indien DISCONNECTED message en niet aan het laden
        if (last_connection_msg_ == "DISCONNECTED" && !charge_connected)
        {
            std::cout << "[CheckNetworkError] NETWORK ERROR -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;  // falback wordt getriggerd, BT komt in ConnectionLost
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckNetworkError] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::string last_connection_msg_;
};


class BatteryOk : public BT::StatefulActionNode
{
public:
    BatteryOk(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_BatteryOk_node");

        rclcpp::QoS qos(1);
        qos.reliable();
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if(msg->data.size() < 2){
                    std::cout << "[CALLBACK] Bericht te kort -> negeren" << std::endl;
                    return;
                }

                int msg_id = msg->data[0] - '0';
                std::string event = msg->data.substr(1);

                std::cout << "[CALLBACK] Parsed msg_id: " << msg_id 
                          << " | event: " << event << std::endl;

                int bt_id = 0;
                if(!getInput("chargingInteger_nextCycle", bt_id)){
                    std::cout << "[CALLBACK] FOUT: kon chargingInteger_nextCycle niet uit blackboard halen!" << std::endl;
                    return;
                } else {
                    std::cout << "[CALLBACK] Blackboard chargingInteger_nextCycle: " << bt_id << std::endl;
                }


                last_event_ = event;
                std::cout << "[CALLBACK] Event geaccepteerd: " << last_event_ << std::endl;

            });

        force_charge_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/force_charge", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("robotLocationBAT"),
            BT::InputPort<int>("chargingInteger_nextCycle"),
            BT::OutputPort<int>("chargingInteger"),
            BT::InputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("skip_drive2charging")
        };
    }

    int updateChargingCounter()
    {
        int counter = 0;

        if (!getInput("chargingInteger_nextCycle", counter))
        {
            std::cout << "[BatteryOk] FOUT BIJ OPHALEN VAN NIEUWE INTEGER UIT BLACKBOARD " << std::endl;
            counter = 0;
        }

        setOutput("chargingInteger", counter);
        return counter;
    }

    // Helper functie om skip_drive2charging te zetten
    void updateSkipDrive()
    {
        std::string admin_status;
        if(getInput("bat_admin_status", admin_status) && admin_status == "START")
        {
            setOutput("skip_drive2charging", true);
        }
        else
        {
            setOutput("skip_drive2charging", false);
        }
    }

    BT::NodeStatus onStart() override
    {
        rclcpp::spin_some(node_);

        std::string bat_state;
        if (getInput("robotLocationBAT", bat_state) && bat_state == "FORCE-CHARGING")
        {
            std::cout << "[BatteryOk] FORCE-CHARGING detected -> sending START" << std::endl;
            int counter = updateChargingCounter();

            std_msgs::msg::String msg;
            msg.data = std::to_string(counter) + "START";

            for (int i = 0; i < 3; ++i)
                force_charge_pub_->publish(msg);

            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        if (last_event_ == "BATTERY-LOW")
        {
            setOutput("chargingInteger", 0);  // BATTERY-LOW bericht komt altijd met 0 voor (want telkens nieuwe sessie)
            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        std::string bat_state;
        if (getInput("robotLocationBAT", bat_state) && bat_state == "FORCE-CHARGING")
        {
            int counter = updateChargingCounter();
            std_msgs::msg::String msg;
            msg.data = std::to_string(counter) + "START";

            for (int i = 0; i < 3; ++i)
                force_charge_pub_->publish(msg);

            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        if (last_event_ == "BATTERY-LOW")
        {
            int getal = updateChargingCounter();
            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    std::string last_event_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_charge_pub_;
};



struct DaySchedule {
    char dayCode;      // M, D, W, T, F, S, U
    bool isActive;     // True als er tijden zijn, False bij 'XXXXXXXX'
    int startTime;     // bijv. 0900
    int endTime;       // bijv. 1700
}; 


class ScheduleParser {
public:
    static std::vector<DaySchedule> getFullSchedule() {
        std::vector<DaySchedule> scheduleList;
        // Zorg dat dit pad exact overeenkomt met waar je Python script schrijft
        std::string filePath = "/home/wheeltec/wheeltec_ros2/src/quiz_bt_node/schedule.txt";  
        std::ifstream file(filePath);
        std::string line;

        if (!file.is_open()) {
            // Log eventueel een error als het bestand niet gevonden wordt
            return scheduleList; 
        }

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == 'R') continue; // skip laatste regel

            if (line.length() < 9) continue; // beveiliging tegen te korte regels

            DaySchedule ds;
            ds.dayCode = line[0];

            if (line.substr(1,8) == "XXXXXXXX") {
                ds.isActive = false;
                ds.startTime = 0;
                ds.endTime = 0;
            } else {
                ds.isActive = true;
                ds.startTime = std::stoi(line.substr(1,4));
                ds.endTime = std::stoi(line.substr(5,4));
            }
            scheduleList.push_back(ds);
        }
                
        file.close();
        return scheduleList;
    }
};


class CheckInWorkingZone : public BT::SyncActionNode
{
public:
    CheckInWorkingZone(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btInWorkingZone");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);

    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("robotLocation"),
            // Blackboard output voor andere nodes
            BT::OutputPort<std::string>("robotLocationBAT")
        };
    }

    BT::NodeStatus tick() override
    {
        // 1. Haal de huidige tijd en dag op
        std::time_t now = std::time(nullptr);
        std::tm *local = std::localtime(&now);

        // tm_wday: 0 = Sun, 1 = Mon, 2 = Tue, 3 = Wed, 4 = Thu, 5 = Fri, 6 = Sat
        // We mappen dit naar jouw dag-codes uit het Python script
        char day_codes[] = {'U', 'M', 'D', 'W', 'T', 'F', 'S'};
        char current_day_code = day_codes[local->tm_wday];
        
        // Huidige tijd in HHMM format (bijv. 14:30 -> 1430)
        int current_time_val = (local->tm_hour * 100) + local->tm_min;

        // 2. Haal het schema op via de static helper
        auto schedule = ScheduleParser::getFullSchedule();
        // --- DEBUG PRINTS START ---
        std::cout << "\n--- Ingelezen Schema ---" << std::endl;
        if (schedule.empty()) {
            std::cout << "[WAARSCHUWING] Schema is leeg! Kan bestand niet lezen." << std::endl;
        } else {
            for (const auto& day : schedule) {
                std::cout << "Dag: " << day.dayCode;
                if (day.isActive) {
                    std::cout << " | Status: ACTIEF | Tijd: " << day.startTime << " tot " << day.endTime << std::endl;
                } else {
                    std::cout << " | Status: INACTIEF" << std::endl;
                }
            }
        }
        bool is_working_time = false;
        for (const auto& day : schedule) {
            if (day.dayCode == current_day_code) {
                if (day.isActive && current_time_val >= day.startTime && current_time_val < day.endTime) {
                    is_working_time = true;
                }
                break; // Dag gevonden, stop met zoeken
            }
        }

        std_msgs::msg::String bt_msg;
        std_msgs::msg::String rpi_msg;

        // 3. Afhandeling op basis van schema
        if (!is_working_time)
        {
            std::cout << "[CheckInWorkingZone] Buiten werkuren (Schedule) -> FORCE-CHARGING" << std::endl;

            setOutput("robotLocationBAT", std::string("FORCE-CHARGING"));

            bt_msg.data = "FORCE-CHARGING";
            pub_->publish(bt_msg);

            // We geven SUCCESS terug omdat de conditie "afgehandeld" is (robot moet gaan laden)
            return BT::NodeStatus::SUCCESS;
        }

        // Als we hier komen, is het werktijd
        setOutput("robotLocationBAT", std::string("WORKING"));

        bt_msg.data = "CheckInWorkingZone-WORKING";
        pub_->publish(bt_msg);

        rpi_msg.data = "RobotExplore"; // Scherm triggeren
        rpi_pub_->publish(rpi_msg);


        // 4. Check of de fysieke locatie ook klopt
        std::string location;
        if (!getInput("robotLocation", location))
        {
            std::cerr << "[CheckInWorkingZone] Geen robotLocation gevonden op blackboard!\n";
            return BT::NodeStatus::FAILURE;
        }

        if (location == "WORKING")
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;
};



class DriveToChargingStation : public BT::StatefulActionNode
{
public:
    DriveToChargingStation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          success_received_(false), timeout_(5.0)
    {
        node_ = rclcpp::Node::make_shared("btDriveToChargingStation");
        rclcpp::QoS qos(1);
        qos.reliable();

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::cout << "\n[CALLBACK] Nieuw bericht ontvangen: " << msg->data << std::endl;

                if(msg->data.size() < 2){
                    std::cout << "[CALLBACK] Bericht te kort -> negeren" << std::endl;
                    return;
                }

                int msg_id = msg->data[0] - '0';
                std::string event = msg->data.substr(1);

                std::cout << "[CALLBACK] Parsed msg_id: " << msg_id 
                          << " | event: " << event << std::endl;

                int bt_id = 0;
                if(!getInput("chargingInteger", bt_id)){
                    std::cout << "[CALLBACK] FOUT: kon chargingInteger niet uit blackboard halen!" << std::endl;
                } else {
                    std::cout << "[CALLBACK] Blackboard chargingInteger: " << bt_id << std::endl;
                }

                if(msg_id != bt_id){
                    std::cout << "[CALLBACK] msg_id != bt_id -> bericht genegeerd" << std::endl;
                    return;
                }

                std::cout << "[CALLBACK] msg_id komt overeen met bt_id!" << std::endl;

                if (event == "DRIVING-TO-DOCK"){
                    std::cout << "[CALLBACK] Event is DRIVING-TO-DOCK -> success_received_ = true" << std::endl;
                    success_received_ = true;
                }
                else{
                    std::cout << "[CALLBACK] Event is iets anders -> success_received_ = false" << std::endl;
                    success_received_ = false;
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<int>("chargingInteger"),
            BT::OutputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("connection_chargeStatus"),  
            BT::InputPort<bool>("skip_drive2charging")  
        };
    }

    BT::NodeStatus onStart() override
    {
        success_received_ = false;
        start_time_ = std::chrono::steady_clock::now();
        getInput("timeout", timeout_);

        setOutput("connection_chargeStatus", true);

        // Check skip_drive2charging
        bool skip = false;
        if (getInput("skip_drive2charging", skip))
        {
            if (skip)
            {
                std::cout << "[DriveToChargingStation] skip_drive2charging = TRUE -> direct SUCCESS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "[DriveToChargingStation] skip_drive2charging = FALSE -> direct FAILURE" << std::endl;
            }
        }

        setOutput("bat_admin_status", "STOP");

        int bt_id = 0;
        getInput("chargingInteger", bt_id);

        std::cout << "\n[DriveToChargingStation] === ON START ===" << std::endl;
        std::cout << "[DriveToChargingStation] timeout: " << timeout_ << std::endl;
        std::cout << "[DriveToChargingStation] chargingInteger: " << bt_id << std::endl;

        std_msgs::msg::String msg;
        msg.data = "DriveToChargingStation";
        pub_->publish(msg);

        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotGoCharge";
        pub_quiz_->publish(quiz_msg);

        std::cout << "[DriveToChargingStation] START waiting for DRIVING-TO-DOCK" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        std::cout << "[DriveToChargingStation] success_received_: " 
                  << (success_received_ ? "TRUE" : "FALSE") << std::endl;

        if (success_received_)
        {
            std::cout << "[DriveToChargingStation] -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        std::cout << "[DriveToChargingStation] elapsed: " << elapsed 
                  << " / timeout: " << timeout_ << std::endl;

        if (elapsed >= timeout_)
        {
            std::cout << "[DriveToChargingStation] -> TIMEOUT FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[DriveToChargingStation] HALTED" << std::endl;
    }

private:
    bool success_received_;
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_; 
};

// Als er iets misloopt met het laden wordt deze node bereikt (via GoToChargeFallback)
// Boom moet herstarten om hier uit te geraken (bv via adminpaneel)
class BatteryStopDrive : public BT::StatefulActionNode
{
public:
    BatteryStopDrive(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btBatteryStopDrive");

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

        
        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};  
    }

    BT::NodeStatus onStart() override
    {
        std_msgs::msg::String msg;
        msg.data = "BatteryStopDrive";
        pub_->publish(msg);

        // NIEUW: scherm triggeren via RPi topic
        std_msgs::msg::String rpi_msg;
        rpi_msg.data = "RobotErrorCharge";
        rpi_pub_->publish(rpi_msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::cout << "[BatteryStopDrive] BEREIKT" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[BatteryStopDrive] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // NIEUW
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;
};


class DriveWorkArea : public BT::StatefulActionNode
{
public:
    DriveWorkArea(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btDriveWorkArea");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        pub_coord_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/btDriveCoord", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("z"),
            BT::InputPort<double>("qx"),
            BT::InputPort<double>("qy"),
            BT::InputPort<double>("qz"),
            BT::InputPort<double>("qw"),
            BT::OutputPort<std::string>("workarea_timestamp")
        };
    }

    BT::NodeStatus onStart() override
    {
        // Publish BT node naam
        std_msgs::msg::String bt_msg;
        bt_msg.data = "DriveWorkArea";
        pub_bt_->publish(bt_msg);


        sent_coord_.header.stamp = node_->get_clock()->now();
        sent_coord_.header.frame_id = "map";

        
        double x, y, z, qx , qy, qz, qw;
        getInput("x", x);
        getInput("y", y);
        getInput("z", z);

        getInput("qx", qx);
        getInput("qy", qy);
        getInput("qz", qz);
        getInput("qw", qw);


        sent_coord_.pose.position.x = x;
        sent_coord_.pose.position.y = y;
        sent_coord_.pose.position.z = z;

        sent_coord_.pose.orientation.x = qx;
        sent_coord_.pose.orientation.y = qy;
        sent_coord_.pose.orientation.z = qz;
        sent_coord_.pose.orientation.w = qw;

        pub_coord_->publish(sent_coord_);

        // Timestamp opslaan
        sent_timestamp_ = std::to_string(sent_coord_.header.stamp.sec) + "." +
                          std::to_string(sent_coord_.header.stamp.nanosec);

        setOutput("workarea_timestamp", sent_timestamp_);

        std::cout << "[DriveWorkArea] Published coordinate at timestamp: "
                  << sent_timestamp_ << std::endl;

        // Timeout ophalen
        if (!getInput<double>("timeout", timeout_))
            timeout_ = 5.0;

        start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time_)
                .count();

        if (elapsed >= timeout_)
        {
            std::cout << "[DriveWorkArea] Timeout ("
                      << timeout_ << "s) -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[DriveWorkArea] HALTED" << std::endl;
    }

private:
    double timeout_;
    std::string sent_timestamp_;
    std::chrono::steady_clock::time_point start_time_;
    geometry_msgs::msg::PoseStamped sent_coord_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_coord_;
};


class RobotExplore : public BT::SyncActionNode
{
public:
    RobotExplore(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
    node_ = rclcpp::Node::make_shared("btRobotExplore");
    pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);
    pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

    }

        static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("robotLocation") };
    }

    BT::NodeStatus tick() override {

        setOutput("robotLocation", "WORKING");


    	std::string state = "RobotExplore";
    	std_msgs::msg::String msg;
        msg.data = state;
        pub_quiz_->publish(msg);
        
        std::string bt_state = "RobotExplore";
        std_msgs::msg::String bt_msg;
        bt_msg.data = bt_state;
        pub_bt_->publish(bt_msg);

        std::cout << "[RobotExplore] Exploring environment (sim)" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;  // bestaande publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;    // nieuwe publisher
};






class StatusDriveToChargingDock : public BT::StatefulActionNode
{
public:
    StatusDriveToChargingDock(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          status_(""), timeout_(5.0)
    {
        node_ = rclcpp::Node::make_shared("btStatusDriveToChargingDock");
        rclcpp::QoS qos(1);
        qos.reliable();
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if(msg->data.size() < 2) return;

                int msg_id = msg->data[0] - '0';
                std::string event = msg->data.substr(1);

                int bt_id = 0;
                getInput("chargingInteger", bt_id);

                if(msg_id != bt_id)
                    return;

                status_ = event;
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<int>("chargingInteger"),
            BT::InputPort<bool>("skip_drive2charging") // <-- nieuw
        };
    }

    BT::NodeStatus onStart() override
    {
        status_ = "";
        getInput("timeout", timeout_);
        start_time_ = std::chrono::steady_clock::now();

        // ✅ check skip_drive2charging bij start
        bool skip = false;
        if (getInput("skip_drive2charging", skip))
        {
            if (skip)
            {
                std::cout << "[StatusDriveToChargingDock] skip_drive2charging = TRUE -> direct SUCCESS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "[StatusDriveToChargingDock] skip_drive2charging = FALSE -> direct FAILURE" << std::endl;
            }
        }

        std_msgs::msg::String msg;
        msg.data = "StatusDriveToChargingDock";
        pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (status_ == "DRIVE-TO-DOCK-SUCCESS")
        {
            std::cout << "[StatusDriveToChargingDock] SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else if (status_ == "DRIVE-TO-DOCK-FAILED" || status_ == "DRIVE-TO-DOCK-CANCELED")
        {
            std::cout << "[StatusDriveToChargingDock] FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed >= timeout_)
        {
            std::cout << "[StatusDriveToChargingDock] Timeout -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        std::cout << "[StatusDriveToChargingDock] HALTED" << std::endl;
    }

private:
    std::string status_;
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class IsRobotCharging : public BT::StatefulActionNode
{
public:
    IsRobotCharging(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          event_(""), timeout_(200.0)
    {
        node_ = rclcpp::Node::make_shared("btIsRobotCharging");

        rclcpp::QoS qos(1);
        qos.reliable();
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                int msg_id = msg->data[0] - '0';
                std::string event = msg->data.substr(1);

                int bt_id = 0;
                getInput("chargingInteger", bt_id);

                if(msg_id != bt_id)
                    return;

                event_ = event;
            });


        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout"),
                BT::InputPort<int>("chargingInteger"),
                BT::InputPort<bool>("skip_drive2charging")   
        };
    }

    BT::NodeStatus onStart() override
    {
        event_ = "";
        getInput("timeout", timeout_);
        start_time_ = std::chrono::steady_clock::now();
        std_msgs::msg::String msg;
        msg.data = "IsRobotCharging";
        pub_->publish(msg);

        bool skip = false;
        if (getInput("skip_drive2charging", skip))
        {
            if (skip)
            {
                std::cout << "[IsRobotCharging] skip_drive2charging = TRUE -> direct SUCCESS" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "[IsRobotCharging] skip_drive2charging = FALSE -> " << std::endl;
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (event_ == "DOCKING-FAILED")
            return BT::NodeStatus::FAILURE;

        if (event_ == "ROBOT-CHARGING")
            return BT::NodeStatus::SUCCESS;

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed >= timeout_)
        {
            std::cout << "[IsRobotCharging] Timeout reached -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override {
        std::cout << "[IsRobotCharging] HALTED" << std::endl;
    }

private:
    std::string event_;
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


class IsBatteryFull : public BT::StatefulActionNode
{
public:
    IsBatteryFull(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btBatteryFull");
        rclcpp::QoS qos(1);
        qos.reliable();
        // Subscriber
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                int msg_id = msg->data[0] - '0';
                std::string event = msg->data.substr(1);

                int bt_id = 0;
                getInput("chargingInteger", bt_id);

                if(msg_id != bt_id)
                    return;

                last_event_ = event;
            });

        // Publisher naar rpitopic
        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("robotLocation"),
                 BT::InputPort<int>("chargingInteger"),
                 BT::OutputPort<std::string>("bat_admin_status")
        };
    }

    BT::NodeStatus onStart() override
    {
        setOutput("robotLocation", "CHARGING");
        setOutput("bat_admin_status", "START");

        // Stuur bericht naar rpitopic
        std_msgs::msg::String msg;
        msg.data = "RobotCharging";
        pub_quiz_->publish(msg);

        std_msgs::msg::String bt_msg;
        bt_msg.data = "IsBatteryFull";
        pub_bt_->publish(bt_msg);

        return BT::NodeStatus::SUCCESS; // altijd succes geven op dit moment voor testing 

        std::cout << "[IsBatteryFull] RobotCharging bericht verzonden" << std::endl;

        rclcpp::spin_some(node_);

        if (last_event_ == "BATTERY-FULL")
        {
            std::cout << "[IsBatteryFull] CHARGING COMPLETED -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (last_event_ == "BATTERY-FULL")
        {
            std::cout << "[IsBatteryFull] CHARGING COMPLETED -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[IsBatteryFull] HALTED" << std::endl;
    }

private:
    std::string last_event_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;

};

class CheckingNearbyVisitors : public BT::StatefulActionNode
{
public:
    CheckingNearbyVisitors(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), success_count_(0)
    {
        node_ = rclcpp::Node::make_shared("btCheckingNearbyVisitors");

        // Publisher (zoals de andere nodes)
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/target_distance", qos,
            [this](std_msgs::msg::Float32::SharedPtr msg) { latest_value_ = msg->data; });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {

        success_count_ = 0;
        latest_value_ = 999.0; 
        std_msgs::msg::String msg;
        msg.data = "CheckingNearbyVisitors";
        pub_->publish(msg);
        std::cout << "[CheckingNearbyVisitors] START" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        rclcpp::spin_some(node_);
        std::cout << "[CheckingNearbyVisitors] Measured distance: " << latest_value_ << std::endl;

        if (latest_value_ <= 1.5 and latest_value_ > 0.05) 
        {
            std::cout << "[CheckingNearbyVisitors] STOP BIJ PERSOON -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckingNearbyVisitors] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    float latest_value_;
    int success_count_;
};
 

// BT node die bepaalt of robot de visitors bereikt heeft via afstand + trigger event
class ArrivedAtVisitors : public BT::StatefulActionNode
{
public:
    ArrivedAtVisitors(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(15.0),
          received_drive_to_quiz_(false),
          overlimit_count_(0),
          follow_value_(0.0)
    {
        node_ = rclcpp::Node::make_shared("btArrivedAtVisitors");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        // volgt afstand tot persoon/target
        sub_follow_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/target_distance",
            10,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                follow_value_ = msg->data;
            });

        rclcpp::QoS qos(1);
        qos.reliable();

        // externe trigger dat robot effectief op quiz locatie is aangekomen
        sub_quiz_ = node_->create_subscription<std_msgs::msg::String>(
            "/quiz",
            qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "drive_to_quiz_location")
                {
                    std::cout << "[ArrivedAtVisitors] Received 'drive_to_quiz_location'" << std::endl;
                    received_drive_to_quiz_ = true;
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("timeout")};
    }

    BT::NodeStatus onStart() override
    {
        overlimit_count_ = 0;
        received_drive_to_quiz_ = false;
        follow_value_ = 0.0;

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 15.0;

        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg_bt_;
        msg_bt_.data = "ArrivedAtVisitors";
        pub_bt_->publish(msg_bt_);

        std::cout << "[ArrivedAtVisitors] START (timeout="
                  << timeout_ << "s)" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        // status wordt continu gepubliceerd voor debugging
        std_msgs::msg::String msg_bt_;
        msg_bt_.data = "ArrivedAtVisitors";
        pub_bt_->publish(msg_bt_);

        // directe success trigger via extern event
        if (received_drive_to_quiz_)
        {
            std::cout << "[ArrivedAtVisitors] 'drive_to_quiz_location' ontvangen -> SUCCESS" << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        // filter: detectie van "te ver weg" over meerdere samples
        if (follow_value_ > 3.0)
        {
            overlimit_count_++;
        }
        else
        {
            overlimit_count_ = 0;
        }

        // robuuste failure als meerdere opeenvolgende metingen te ver zijn
        if (overlimit_count_ >= 5)
        {
            std::cout << "[ArrivedAtVisitors] 5 metingen > 3.0 -> FAILURE" << std::endl;

            return BT::NodeStatus::FAILURE;
        }

        // timeout fallback
        auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start_time_)
                           .count();

        if (elapsed >= timeout_)
        {
            std::cout << "[ArrivedAtVisitors] Timeout (" << elapsed
                      << "s) -> FAILURE" << std::endl;

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[ArrivedAtVisitors] HALTED" << std::endl;
    }

private:
    double timeout_;
    bool received_drive_to_quiz_;
    int overlimit_count_;
    float follow_value_;

    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_follow_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_quiz_;
};



// BT node die een doelpositie naar quiz locatie stuurt via PoseStamped
class DriveQuizLocation : public BT::StatefulActionNode
{
public:
    DriveQuizLocation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btDriveQuizLocation");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        pub_coord_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/btDriveCoord", 10);

        pub_tracking_enable_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/tracking_enable", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("z"),
            BT::InputPort<double>("qx"),
            BT::InputPort<double>("qy"),
            BT::InputPort<double>("qz"),
            BT::InputPort<double>("qw"),
            BT::OutputPort<std::string>("sent_timestamp")
        };
    }

    BT::NodeStatus onStart() override
    {
        std_msgs::msg::String bt_msg;
        bt_msg.data = "DriveQuizLocation";
        pub_bt_->publish(bt_msg);

        // stop follow-me/person tracking
        std_msgs::msg::Bool tracking_msg;
        tracking_msg.data = false;
        pub_tracking_enable_->publish(tracking_msg);

        std::cout << "[DriveQuizLocation] Tracking DISABLED" << std::endl;

        // pose opbouwen voor navigation stack
        sent_coord_.header.stamp = node_->get_clock()->now();
        sent_coord_.header.frame_id = "map";

        double x, y, z, qx, qy, qz, qw;
        getInput("x", x);
        getInput("y", y);
        getInput("z", z);

        getInput("qx", qx);
        getInput("qy", qy);
        getInput("qz", qz);
        getInput("qw", qw);

        sent_coord_.pose.position.x = x;
        sent_coord_.pose.position.y = y;
        sent_coord_.pose.position.z = z;

        sent_coord_.pose.orientation.x = qx;
        sent_coord_.pose.orientation.y = qy;
        sent_coord_.pose.orientation.z = qz;
        sent_coord_.pose.orientation.w = qw;

        pub_coord_->publish(sent_coord_);

        // timestamp voor tracking van deze navigation request
        sent_timestamp_ = std::to_string(sent_coord_.header.stamp.sec) + "." +
                          std::to_string(sent_coord_.header.stamp.nanosec);

        setOutput("sent_timestamp", sent_timestamp_);

        std::cout << "[DriveQuizLocation] Published coordinate at timestamp: "
                  << sent_timestamp_ << std::endl;

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 5.0;

        start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed >= timeout_)
        {
            std::cout << "[DriveQuizLocation] Timeout (" << timeout_ << "s) -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[DriveQuizLocation] HALTED" << std::endl;
    }

private:
    double timeout_;
    std::string sent_timestamp_;
    std::chrono::steady_clock::time_point start_time_;
    geometry_msgs::msg::PoseStamped sent_coord_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_coord_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tracking_enable_;
};




class IsRobotAtQuiz : public BT::StatefulActionNode
{
public:
    IsRobotAtQuiz(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), timeout_(10.0)
    {
        node_ = rclcpp::Node::make_shared("btIsRobotAtQuiz");

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/drive_to_coord_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string data = msg->data;
                std::cout << "[IsRobotAtQuiz] Ontvangen bericht: " << data << std::endl;

                // Split op '-'
                std::vector<std::string> parts;
                std::stringstream ss(data);
                std::string segment;
                while (std::getline(ss, segment, '-'))
                {
                    parts.push_back(segment);
                }

                if (parts.size() < 2)
                    return;

                std::string status_code = parts[0];
                std::string recv_timestamp = parts[1];

                // Alleen eerste 10 cijfers van timestamp vergelijken
                std::string expected_prefix = sent_timestamp_.substr(0, 10);
                std::string recv_prefix = recv_timestamp.substr(0, 10);

                if (recv_prefix == expected_prefix)
                {
                    if (status_code == "04")
                        received_success_ = true;
                    else if (status_code == "05" ||  status_code == "07"){
                        received_failure_ = true;
                        std::cout << "FAILURE ONTVANGEN";
                    }
                   
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("sent_timestamp"),  // Timestamp uit blackboard
            BT::OutputPort<bool>("drive_failed")
              
        };
    }

    BT::NodeStatus onStart() override
    {
        //return BT::NodeStatus::SUCCESS;

        received_success_ = false;
        received_failure_ = false;
        start_time_ = std::chrono::steady_clock::now();

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 10.0;

        if (!getInput<std::string>("sent_timestamp", sent_timestamp_))
            std::cout << "[IsRobotAtQuiz] Geen timestamp ontvangen van blackboard!" << std::endl;
        else
            std::cout << "[IsRobotAtQuiz] Verwachte timestamp = " << sent_timestamp_ << std::endl;

        // BT node naam publiceren
        std_msgs::msg::String msg;
        msg.data = "IsRobotAtQuiz";
        pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {


        rclcpp::spin_some(node_);
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

        if (received_success_)
        {
            std::cout << "[IsRobotAtQuiz] Successtatus ontvangen -> SUCCESS" << std::endl;
            setOutput("drive_failed", false);

            return BT::NodeStatus::SUCCESS;
        }

        if (received_failure_)
        {
            std::cout << "[IsRobotAtQuiz] Faalstatus ontvangen -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (elapsed >= timeout_)
        {
            std::cout << "[IsRobotAtQuiz] Timeout (" << timeout_ << "s) -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[IsRobotAtQuiz] HALTED" << std::endl;
    }

private:
    double timeout_;
    bool received_success_;
    bool received_failure_;
    std::chrono::steady_clock::time_point start_time_;
    std::string sent_timestamp_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};



class CheckButtonState : public BT::SyncActionNode
{
public:
    CheckButtonState(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btCheckButtonState");

        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            
            BT::OutputPort<bool>("buttonStop"),
            BT::OutputPort<std::string>("robotLocationBAT")
        };
    }

    BT::NodeStatus tick() override
    {
        // 1. Publiceer node naam
        std_msgs::msg::String msg;
        msg.data = "CheckButtonState";
        pub_->publish(msg);

        // 2. Lees laatste lijn van file
        std::string file_path = "/home/wheeltec/wheeltec_ros2/src/quiz_bt_node/schedule.txt";
        std::ifstream file(file_path);

        if (!file.is_open())
        {
            std::cerr << "[CheckButtonState] Kan file niet openen!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::string line, last_line;

        while (std::getline(file, line))
        {
            if (!line.empty())
                last_line = line;
        }

        file.close();

        std::cout << "[CheckButtonState] Laatste lijn: " << last_line << std::endl;

        // 3. Parse ROBOTACTIVE
        bool robot_active;

        if (last_line.find("ROBOTACTIVE:true") != std::string::npos)
        {

            std::cout << "[RobotActiveTrue] Laatste lijn TRUE: " << std::endl;

            robot_active = true;
        }
        else if (last_line.find("ROBOTACTIVE:false") != std::string::npos)
        {
            std::cout << "[RobotActiveTrue] Laatste lijn FALSE: " << std::endl;

            robot_active = false;
        }
        else
        {
            std::cerr << "[CheckButtonState] Onbekend formaat!" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // 4. Gedrag (vervanging van START/STOP button)
        if (robot_active)
        {
            // === STARTBUTTON gedrag ===
            std::cout << "[CheckButtonState] START toestand (robot actief)" << std::endl;
            
            setOutput("buttonStop", false);
         
        }
        else
        {
            // === STOPBUTTON gedrag ===
            std::cout << "[CheckButtonState] STOP toestand (robot NIET actief)" << std::endl;

            setOutput("buttonStop", true);
            setOutput("robotLocationBAT", std::string("FORCE-CHARGING"));
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


class RobotWaitInChargingStation : public BT::StatefulActionNode
{
public:
    RobotWaitInChargingStation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btRobotWaitInChargingStation");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("buttonStop")  
        };
    }

    BT::NodeStatus onStart() override
    {
        std_msgs::msg::String msg;
        msg.data = "RobotWaitInChargingStation";
        pub_->publish(msg);
        std::cout << "[RobotWaitInChargingStation] ONSTART" << std::endl;

        return checkConditions();
    }

    BT::NodeStatus onRunning() override
    {
        return checkConditions();
    }

    void onHalted() override
    {
        std::cout << "[RobotWaitInChargingStation] HALTED" << std::endl;
    }

private:
    BT::NodeStatus checkConditions()
    {
        bool buttonStop = false;
        if (!getInput("buttonStop", buttonStop))
        {
            //std::cerr << "[RobotWaitInChargingStation] Kan buttonStop niet lezen, default false" << std::endl;
        }

        if (buttonStop)
        {
            std::cout << "[RobotWaitInChargingStation] buttonStop true " << std::endl;

            return BT::NodeStatus::RUNNING;
        }

        // Check tijdschema alleen als buttonStop false is
        return checkTime();
    }

    BT::NodeStatus checkTime()
    {
        // 1. Huidige tijd en dag bepalen
        std::time_t now = std::time(nullptr);
        std::tm *local = std::localtime(&now);

        char day_codes[] = {'U', 'M', 'D', 'W', 'T', 'F', 'S'};
        char current_day_code = day_codes[local->tm_wday];
        int current_time_val = (local->tm_hour * 100) + local->tm_min;

        // 2. Schema ophalen
        auto schedule = ScheduleParser::getFullSchedule();
        
        bool is_working_time = false;
        for (const auto& day : schedule) {
            if (day.dayCode == current_day_code) {
                if (day.isActive && current_time_val >= day.startTime && current_time_val < day.endTime) {
                    is_working_time = true;
                }
                break;
            }
        }

        // 3. Logica voor de State
        if (is_working_time)
        {
            std::cout << "[RobotWaitInChargingStation] Werktijd begonnen (" << current_time_val << ") -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        //std::cout << "[RobotWaitInChargingStation] Nog geen werktijd "<< std::endl;

        // Nog geen werktijd, blijf RUNNING
        return BT::NodeStatus::RUNNING;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};



class StopRobotCharging : public BT::StatefulActionNode
{
public:
    StopRobotCharging(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btStopRobotCharging");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        force_charge_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/force_charge", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("chargingInteger"),
                    BT::OutputPort<int>("chargingInteger_nextCycle") };

    }

    int incrementChargingCounter() {
        int counter = 0;

        if (!getInput("chargingInteger", counter))
        {
            throw BT::RuntimeError("chargingInteger ontbreekt");
        }

        if (counter == 9){
            counter = 0;
        }
        else{
            counter += 1;
        }
        
        std::cout << "[StopRobotCharging] regel voor setoutput chargingintegernextcycle" << std::endl;

        setOutput("chargingInteger_nextCycle", counter);

        return counter;
    }

    BT::NodeStatus onStart() override
    {
        // Publish BT node naam
        std_msgs::msg::String bt_msg;
        bt_msg.data = "StopRobotCharging";
        pub_bt_->publish(bt_msg);

        int charging_counter_new = incrementChargingCounter();

        // Publish STOP command
        int charge_id = 0;
        getInput("chargingInteger", charge_id);

        std_msgs::msg::String cmd_msg;
        cmd_msg.data = std::to_string(charge_id) + "STOP";

        for (int i = 0; i < 3; ++i)
        {
            force_charge_pub_->publish(cmd_msg);
        }

        std::cout << "[StopRobotCharging] Published STOP to /force_charge" << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning() override
    {
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        std::cout << "[StopRobotCharging] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_charge_pub_;
};

class RobotAtQuiz : public BT::SyncActionNode
{
public:
    RobotAtQuiz(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {
        node_ = rclcpp::Node::make_shared("bt_robot_at_quiz_node");

        // Bestaande publisher behouden
        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);

        // Nieuwe publisher voor BehaviorTree-node status
        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

        static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("robotLocation") };
    }


    BT::NodeStatus tick() override {

        setOutput("robotLocation", "QUIZ");

        std::string state = "robot-arrived-at-quiz-location";
        std_msgs::msg::String msg;
        msg.data = state;
        pub_quiz_->publish(msg);


        std::string bt_state = "RobotAtQuiz";
        std_msgs::msg::String bt_msg;
        bt_msg.data = bt_state;
        pub_bt_->publish(bt_msg);

        std::cout << "[RobotAtQuiz] Robot arrived at quiz location, published to /quiz_pi_con and /BehaviorTreeNode" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;  // bestaande publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;    // nieuwe publisher
};


/*/
class MDChargeLocation: public BT::SyncActionNode
{
public:
    MDChargeLocation(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btMDChargeLocation");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_pose_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("z"),
            BT::InputPort<double>("qx"),
            BT::InputPort<double>("qy"),
            BT::InputPort<double>("qz"),
            BT::InputPort<double>("qw"),
        };
    }

    BT::NodeStatus tick() override
    {
        std_msgs::msg::String bt_msg;
        bt_msg.data = "MDChargeLocation";
        pub_bt_->publish(bt_msg);


        double x, y, z, qx, qy, qz, qw;
        if (!getInput("x", x) || !getInput("y", y) || !getInput("z", z) ||
            !getInput("qx", qx) || !getInput("qy", qy) || !getInput("qz", qz) || !getInput("qw", qw))
        {
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

        pose_msg.header.stamp = node_->get_clock()->now();
        pose_msg.header.frame_id = "map";

        // Positie en Orientatie
        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = z;
        pose_msg.pose.pose.orientation.x = qx;
        pose_msg.pose.pose.orientation.y = qy;
        pose_msg.pose.pose.orientation.z = qz;
        pose_msg.pose.pose.orientation.w = qw;

        // Covariance instellen (36 elementen array)
        // We zetten alle waarden op 0.0
        std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);

        // En we geven een hoge zekerheid (kleine getallen) op de belangrijke assen:
        pose_msg.pose.covariance[0] = 0.01;  // X zekerheid
        pose_msg.pose.covariance[7] = 0.01;  // Y zekerheid
        pose_msg.pose.covariance[35] = 0.01; // Yaw (Z-rotatie) zekerheid

        // Publish de pose (1 keer is meestal genoeg, maar 3 kan als je netwerk hapert)
        pub_pose_->publish(pose_msg);

        std::cout << "[MDForceCharging] Pose published naar /initialpose" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
};

/*/


class BatteryCharged : public BT::StatefulActionNode
{
public:
    BatteryCharged(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(10.0)   // default timeout
    {
        node_ = rclcpp::Node::make_shared("btBatteryCharged");

        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout"), 
                BT::OutputPort<std::string>("robotLocationBAT"),
                BT::OutputPort<std::string>("bat_admin_status"),
                 BT::OutputPort<bool>("connection_chargeStatus")  

            };
    }

    BT::NodeStatus onStart() override
    {
        // timeout uit XML of default
        if (!getInput<double>("timeout", timeout_))
            timeout_ = 10.0;


        setOutput("bat_admin_status", "STOP");
        setOutput("connection_chargeStatus", false);

        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "BatteryCharged";
        pub_->publish(msg);

        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotStarting";
        pub_quiz_->publish(quiz_msg);


        std::cout << "[BatteryCharged] START (timeout="
                  << timeout_ << "s)" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_)
                           .count();

        if (elapsed >= timeout_)
        {
            std::cout << "[BatteryCharged] Timeout reached -> SUCCESS"
                      << std::endl;
            setOutput("robotLocationBAT",
                  std::string("RETURN-FROM-CHARGING"));
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[BatteryCharged] HALTED" << std::endl;
    }

private:
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;

};

class IsRobotAtWorkArea : public BT::StatefulActionNode
{
public:
    IsRobotAtWorkArea(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), timeout_(10.0)
    {
        node_ = rclcpp::Node::make_shared("btIsRobotAtWorkArea");

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/drive_to_coord_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string data = msg->data;
                std::cout << "[IsRobotAtWorkArea] Ontvangen bericht: " << data << std::endl;

                if (data.size() >= 7 && data.substr(0, 7) == "12-0000")
                {
                    received_failure_ = true;
                    std::cout << "[IsRobotAtWorkArea] FAILURE door 12-0000 prefix" << std::endl;
                    return;
                }

                std::vector<std::string> parts;
                std::stringstream ss(data);
                std::string segment;
                while (std::getline(ss, segment, '-'))
                {
                    parts.push_back(segment);
                }

                if (parts.size() < 2)
                    return;

                std::string status_code = parts[0];
                std::string recv_timestamp = parts[1];

                std::string expected_prefix = sent_timestamp_.substr(0, 10);
                std::string recv_prefix = recv_timestamp.substr(0, 10);

                if (recv_prefix == expected_prefix)
                {
                    if (status_code == "04")
                    {
                        received_success_ = true;
                    }
                    else if (status_code == "05" || status_code == "07")
                    {
                        received_failure_ = true;
                        std::cout << "[IsRobotAtWorkArea] FAILURE ONTVANGEN" << std::endl;
                    }
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("workarea_timestamp"),
            BT::OutputPort<bool>("drive_failed")
        };
    }

    BT::NodeStatus onStart() override
    {

        received_success_ = false;
        received_failure_ = false;
        start_time_ = std::chrono::steady_clock::now();

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 10.0;

        if (!getInput<std::string>("workarea_timestamp", sent_timestamp_))
            std::cout << "[IsRobotAtWorkArea] Geen timestamp ontvangen van blackboard!" << std::endl;
        else
            std::cout << "[IsRobotAtWorkArea] Verwachte timestamp = " << sent_timestamp_ << std::endl;

        // BT node naam publiceren
        std_msgs::msg::String msg;
        msg.data = "IsRobotAtWorkArea";
        pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (received_success_)
        {
            std::cout << "[IsRobotAtWorkArea] Successtatus ontvangen -> SUCCESS" << std::endl;
            setOutput("drive_failed", false);
            return BT::NodeStatus::SUCCESS;
        }

        if (received_failure_)
        {
            std::cout << "[IsRobotAtWorkArea] Faalstatus ontvangen -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (elapsed >= timeout_)
        {
            std::cout << "[IsRobotAtWorkArea] Timeout (" 
                      << timeout_ << "s) -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[IsRobotAtWorkArea] HALTED" << std::endl;
    }

private:
    double timeout_;
    bool received_success_;
    bool received_failure_;
    std::chrono::steady_clock::time_point start_time_;
    std::string sent_timestamp_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};



/*/
class MDTurnAround : public BT::StatefulActionNode
{
public:
    MDTurnAround(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          angular_speed_(0.3),   // rad/s (traag)
          target_rotations_(3.0)
    {
        node_ = rclcpp::Node::make_shared("btMDTurnAround");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_cmd_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {}; // geen blackboard nodig
    }

    BT::NodeStatus onStart() override
    {
        start_time_ = std::chrono::steady_clock::now();

        // publiceer naam van de node
        std_msgs::msg::String msg;
        msg.data = "MDTurnAround";
        pub_bt_->publish(msg);

        std::cout << "[MDTurnAround] START draaien..." << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        // bereken verstreken tijd
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        // totale hoek = omega * tijd
        double rotated_angle = angular_speed_ * elapsed;

        // 3 rondjes = 3 * 2π
        double target_angle = target_rotations_ * 2.0 * M_PI;

        if (rotated_angle >= target_angle)
        {
            stopRobot();

            std::cout << "[MDTurnAround] Klaar met 3 rotaties -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        // blijf draaien
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed_; // traag draaien

        pub_cmd_->publish(cmd);

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        stopRobot();
        std::cout << "[MDTurnAround] HALTED" << std::endl;
    }

private:
    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_cmd_->publish(cmd);
    }

    double angular_speed_;      // rad/s
    double target_rotations_;   // aantal rondjes
    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
}; 

/*/

class WaitQuizToEnd : public BT::StatefulActionNode
{
public:
    WaitQuizToEnd(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(30.0), received_(false)
    {
        node_ = rclcpp::Node::make_shared("btWaitQuizToEnd");

        // Publisher voor status (zoals de andere nodes)
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);


        rclcpp::QoS qos(1);
        qos.reliable(); 
        // Subscriber om te luisteren naar quizstatus
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/quiz", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "quiz-finished" || msg->data == "quiz-inactive")
                {
                    received_ = true;
                    last_msg_ = msg->data;
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override
    {
        received_ = false;
        last_msg_.clear();

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 30.0; // default 30s

        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "WaitQuizToEnd";
        pub_->publish(msg);

        std::cout << "[WaitQuizToEnd] START waiting for 'quiz_finished' or 'quiz_inactive', timeout=" << timeout_ << "s" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        rclcpp::spin_some(node_);

        if (received_)
        {
            std::cout << "[WaitQuizToEnd] Received '" << last_msg_ << "' -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed >= timeout_)
        {
            std::cout << "[WaitQuizToEnd] Timeout reached after " << elapsed << "s -> FAILURE" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "[WaitQuizToEnd] Still waiting... (" << elapsed << "s)" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[WaitQuizToEnd] HALTED" << std::endl;
    }

private:
    double timeout_;
    bool received_;
    std::string last_msg_;
    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

class MainBTStopDrive : public BT::StatefulActionNode
{
public:
    MainBTStopDrive(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btMainBTStopDrive");

        // Publisher voor BT-status zoals bij andere nodes
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("drive_failed"),  // BlackBoard input
        };
    }

    BT::NodeStatus onStart() override
    {
        // Publiceer de node-status
        std_msgs::msg::String msg;
        msg.data = "MainBTStopDrive";
        pub_->publish(msg);

        // RPI topic publish
        std_msgs::msg::String rpi_msg;
        rpi_msg.data = "RobotError";
        rpi_pub_->publish(rpi_msg);

        

        std::cout << "[MainBTStopDrive] Node gestart, check drive_failed op blackboard" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        bool drive_failed = false;
        if (!getInput<bool>("drive_failed", drive_failed))
        {
            std::cout << "[MainBTStopDrive] Geen drive_failed waarde gevonden, ga van FALSE uit" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        if (drive_failed)
        {
            std::cout << "[MainBTStopDrive] drive_failed = TRUE, blijf RUNNING" << std::endl;
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            std::cout << "[MainBTStopDrive] drive_failed = FALSE, returning SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
    }

    void onHalted() override
    {
        std::cout << "[MainBTStopDrive] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;
};

class MainBTSetErrorFlag : public BT::StatefulActionNode
{
public:
    MainBTSetErrorFlag(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btMainBTSetErrorFlag");

        // Publisher naar rpitopic
        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("drive_failed")  // BlackBoard output
        };
    }

    BT::NodeStatus onStart() override
    {
        // Zet blackboard flag
        setOutput("drive_failed", true);

        // Publish naar RPI topic
        std_msgs::msg::String msg;
        msg.data = "RobotExplore";
        rpi_pub_->publish(msg);

        // Start tijd registreren
        start_time_ = std::chrono::steady_clock::now();

        std::cout << "[MainBTSetErrorFlag] started -> publishing RobotExplore" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

        if (seconds >= 5)
        {
            std::cout << "[MainBTSetErrorFlag] 5 seconds done -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        std::cout << "[MainBTSetErrorFlag] running... (" << seconds << "s)" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[MainBTSetErrorFlag] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;

    std::chrono::steady_clock::time_point start_time_;
};
// =======================================================
// StartDrivingToPeople
// → Start beweging richting bezoekers/people node
// → Publishes BT status + quiz trigger
// =======================================================

class StartDrivingToPeople : public BT::StatefulActionNode
{
public:
    StartDrivingToPeople(const std::string &name,
                         const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(5.0)
    {
        node_ = rclcpp::Node::make_shared("btStartDrivingToPeople");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);

        pub_tracking_enable_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/tracking_enable", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout")
        };
    }

    BT::NodeStatus onStart() override
    {
        getInput("timeout", timeout_);

        start_time_ = std::chrono::steady_clock::now();

        // Publish naar BehaviorTreeNode
        std_msgs::msg::String bt_msg;
        bt_msg.data = "StartDrivingToPeople";
        pub_bt_->publish(bt_msg);

        // Publish naar RPi topic
        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotArrivedAtVisitors";
        pub_quiz_->publish(quiz_msg);

        // Tracking inschakelen
        std_msgs::msg::Bool tracking_msg;
        tracking_msg.data = true;
        pub_tracking_enable_->publish(tracking_msg);

        std::cout << "[StartDrivingToPeople] Tracking ENABLED" << std::endl;
        std::cout << "[StartDrivingToPeople] Started driving to people, timeout="
                  << timeout_ << "s" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time_)
                .count();

        if (elapsed >= timeout_)
        {
            std::cout << "[StartDrivingToPeople] Timeout reached -> SUCCESS"
                      << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // Tracking uitschakelen
        std_msgs::msg::Bool tracking_msg;
        tracking_msg.data = false;
        pub_tracking_enable_->publish(tracking_msg);

        std::cout << "[StartDrivingToPeople] Tracking DISABLED" << std::endl;
        std::cout << "[StartDrivingToPeople] HALTED" << std::endl;
    }

private:
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tracking_enable_;
};




class LoopSequence : public BT::DecoratorNode
{
public:
    LoopSequence(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {

        while (true)
        {

            const BT::NodeStatus child_state = child_node_->executeTick();

            if (child_state == BT::NodeStatus::RUNNING)
            {
                return BT::NodeStatus::RUNNING;
            }
            else if (child_state == BT::NodeStatus::SUCCESS)
            {
                return BT::NodeStatus::SUCCESS; // stop met herhalen
            }
            else if (child_state == BT::NodeStatus::FAILURE)
            {
                std::cout << "[LoopSequence] Child failed, restarting sequence..." << std::endl;
                child_node_->halt();  // reset child
                continue;             // herhaal sequence
            }
        }
    }

    void halt() override
    {
        child_node_->halt();
        setStatus(BT::NodeStatus::IDLE);
    }
};


#include <chrono>

class CheckAdminCondition : public BT::StatefulActionNode
{
public:
    CheckAdminCondition(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          admin_closed_(false),
          manual_drive_(false),
          timer_started_(false)
    {
        node_ = rclcpp::Node::make_shared("btCheckAdminCondition");

        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        force_charge_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/force_charge", 10);

        rclcpp::QoS qos(1);
        qos.reliable();

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/admin", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "ADMINPANELCLOSED")
                {
                    std::cout << "[CheckAdminCondition] ADMINPANELCLOSED ontvangen!" << std::endl;
                    admin_closed_ = true;

                    // start timer
                    timer_started_ = true;
                    start_time_ = std::chrono::steady_clock::now();
                }

            });
    }

    static BT::PortsList providedPorts()
    {
        return {

            BT::InputPort<std::string>("bat_admin_status"),
            BT::InputPort<int>("chargingInteger"),
            BT::OutputPort<int>("chargingInteger_nextCycle")
        };
    }

    int incrementChargingCounter()
    {
        int counter = 0;

        if (!getInput("chargingInteger", counter))
        {
            throw BT::RuntimeError("chargingInteger ontbreekt");
        }

        if (counter == 9)
        {
            counter = 0;
        }
        else
        {
            counter += 1;
        }

        setOutput("chargingInteger_nextCycle", counter);
        return counter;
    }

    void publishStopCommand()
    {
        int charge_id = 0;

        if (!getInput("chargingInteger", charge_id))
        {
            throw BT::RuntimeError("chargingInteger ontbreekt");
        }

        std_msgs::msg::String cmd_msg;
        cmd_msg.data = std::to_string(charge_id) + "STOP";

        for (int i = 0; i < 3; ++i)
        {
            force_charge_pub_->publish(cmd_msg);
        }

        std::cout << "[CheckAdminCondition] STOP gestuurd naar /force_charge" << std::endl;
    }

    BT::NodeStatus onStart() override
    {
        admin_closed_ = false;
        manual_drive_ = false;
        timer_started_ = false;

        std_msgs::msg::String msg;
        msg.data = "CheckAdminCondition";
        pub_->publish(msg);

        // Als bat_status stop is, zit robot niet in laadstation (hij reed er naar toe of was dit zelfs niet van plan)
        // Er wordt 1 stop bericht gestuurd naar autocharge (wat wordt genegeerd indien het niet nodig was)
        // En de integer voor volgende keer is +=1 gegaan
        std::string bat_status;
        if (getInput("bat_admin_status", bat_status))
        {
            if (bat_status == "STOP")
            {
                std::cout << "[CheckAdminCondition] bat_admin_status = STOP" << std::endl;

                incrementChargingCounter();
                publishStopCommand();
            }
        }

        return BT::NodeStatus::RUNNING;
    }



    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);


        // 🔹 Admin closed + 3 sec delay
        if (admin_closed_)
        {
            if (timer_started_)
            {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();

                if (elapsed >= 3)
                {
                    std::cout << "[CheckAdminCondition] Admin panel closed + delay -> SUCCESS" << std::endl;
                  
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckAdminCondition] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_charge_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    bool admin_closed_;
    bool manual_drive_;

    // 🔹 Timer vars
    bool timer_started_;
    std::chrono::steady_clock::time_point start_time_;
};


class CheckAdminPanel : public BT::StatefulActionNode
{
public:
    CheckAdminPanel(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          admin_panel_open_(false)
    {
        node_ = rclcpp::Node::make_shared("btCheckAdminPanel");

        // Subscriber naar /admin topic
        rclcpp::QoS qos(1);
        qos.reliable(); 
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/admin", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "ADMINPANELOPEN")
                {
                    std::cout << "[CheckAdminPanel] ADMINPANELOPEN ontvangen!" << std::endl;
                    admin_panel_open_ = true;
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        admin_panel_open_ = false;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (admin_panel_open_)
        {
            std::cout << "[CheckAdminPanel] Admin panel open -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckAdminPanel] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    bool admin_panel_open_;
};



// -------------------------
// MAIN
// -------------------------
int main(int argc, char **argv)
{

    // starten RO2 communicatie 
    rclcpp::init(argc, argv);

    // aanmaak object
    BT::BehaviorTreeFactory factory;

    // registreer nodes
    // < > geeft de naam van de c++ node, de (" ... ") geeft de naam van de node in de XML file waar je deze c++ code aan koppelt

    factory.registerNodeType<CheckInWorkingZone>("CheckInWorkingZone");
    factory.registerNodeType<DriveWorkArea>("DriveWorkArea");
    factory.registerNodeType<IsRobotAtWorkArea >("IsRobotAtWorkArea");
    factory.registerNodeType<RobotExplore>("RobotExplore");
    factory.registerNodeType<StartDrivingToPeople>("StartDrivingToPeople");
    factory.registerNodeType<CheckingNearbyVisitors>("CheckingNearbyVisitors");

    factory.registerNodeType<ArrivedAtVisitors>("ArrivedAtVisitors");

    factory.registerNodeType<DriveQuizLocation>("DriveQuizLocation");
    factory.registerNodeType<IsRobotAtQuiz>("IsRobotAtQuiz");
    factory.registerNodeType<RobotAtQuiz>("RobotAtQuiz");
    factory.registerNodeType<WaitQuizToEnd>("WaitQuizToEnd");
    factory.registerNodeType<BatteryOk>("BatteryOk");
    factory.registerNodeType<DriveToChargingStation>("DriveToChargingStation");
    factory.registerNodeType<StatusDriveToChargingDock>("StatusDriveToChargingDock");
    factory.registerNodeType<IsRobotCharging>("IsRobotCharging");
    factory.registerNodeType<IsBatteryFull>("IsBatteryFull");
    factory.registerNodeType<BatteryCharged>("BatteryCharged");
    factory.registerNodeType<RobotWaitInChargingStation>("RobotWaitInChargingStation");
    factory.registerNodeType<BatteryStopDrive>("BatteryStopDrive");


    factory.registerNodeType<CheckNetworkError>("CheckNetworkError");
    factory.registerNodeType<CheckAdminPanel>("CheckAdminPanel");

    factory.registerNodeType<CheckAdminCondition>("CheckAdminCondition");
    factory.registerNodeType<CheckButtonState>("CheckButtonState");

    factory.registerNodeType<ConnectionLost >("ConnectionLost");

    factory.registerNodeType<StopRobotCharging>("StopRobotCharging");
    factory.registerNodeType<MainBTStopDrive>("MainBTStopDrive");
    factory.registerNodeType<MainBTSetErrorFlag>("MainBTSetErrorFlag");

    factory.registerNodeType<ForceSuccess>("MainFallbackForceSuccess");
    factory.registerNodeType<ForceSuccess>("BatteryForceSuccess");


    factory.registerNodeType<LoopSequence>("LoopSequence");
  


    // laad boom uit XML
    auto tree = factory.createTreeFromFile("src/mecabot_bt/trees/behavior_tree.xml");

    std::cout << "--- Starting BT in continuous mode ---" << std::endl;
    rclcpp::Rate loop_rate(1.0); // definieer hoeveel ticks/sec naar rootnode gaan

    while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickRoot(); // root ticken

        // status van root succes of failure : ga dan halten
        if (status == BT::NodeStatus::SUCCESS) {
            std::cout << "--- Tree ticked to SUCCESS ---" << std::endl;
            tree.rootNode()->halt();
        }
        else if (status == BT::NodeStatus::FAILURE) {
            std::cout << "--- Tree ticked to FAILURE ---" << std::endl;
            tree.rootNode()->halt();
        }

        loop_rate.sleep(); // wacht zolang zodat de rate klopt
    }

    rclcpp::shutdown();
    return 0;
}

