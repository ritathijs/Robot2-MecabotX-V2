#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/decorator_node.h"
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <std_msgs/msg/float32.hpp> 
#include <std_msgs/msg/bool.hpp>  
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals; 

class TimedCondition : public BT::StatefulActionNode
{
public:
    TimedCondition(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config), timeout_(7.0)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override
    {
        // haal timeout uit XML, default 7s
        if (!getInput<double>("timeout", timeout_))
        {
            timeout_ = 7.0;
        }

        start_time_ = std::chrono::steady_clock::now();
        std::cout << "[" << name() << "] START with timeout = " << timeout_ << "s" << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        std::cout << "[" << name() << "] Running... (" << elapsed << "s)" << std::endl;

        if (elapsed >= timeout_)
        {
            std::cout << "[" << name() << "] Timeout reached " << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[" << name() << "] HALTED" << std::endl;
    }

protected:
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
};


class BatteryOk : public BT::StatefulActionNode
{
public:
    BatteryOk(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_BatteryOk_node");
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                last_event_ = msg->data;
            });
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        rclcpp::spin_some(node_);
        std::cout << "[BatteryOk] START, last_event=" << last_event_ << std::endl;

        if (last_event_ == "BATTERY-LOW")
        {
          //  std::cout << "[BatteryOk] Battery low -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        if (last_event_ == "BATTERY-LOW")
        {
            std::cout << "[BatteryOk] Battery low detected -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        //std::cout << "[BatteryOk] Battery OK -> RUNNING" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        //std::cout << "[BatteryOk] HALTED" << std::endl;
    }

private:
    std::string last_event_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};


class DriveToChargingStation : public BT::StatefulActionNode
{
public:
    DriveToChargingStation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          success_received_(false), timeout_(5.0)
    {
        node_ = rclcpp::Node::make_shared("btDriveToChargingStation");
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                if (msg->data == "DRIVING-TO-DOCK")
                    success_received_ = true;
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override
    {
        success_received_ = false;
        start_time_ = std::chrono::steady_clock::now();
        getInput("timeout", timeout_);
        std_msgs::msg::String msg;
        msg.data = "DriveToChargingStation";
        pub_->publish(msg);
        std::cout << "[DriveToChargingStation] START waiting for DRIVING-TO-DOCK" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        if (success_received_)
        {
            std::cout << "[DriveToChargingStation] Received DRIVING-TO-DOCK -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();

        if (elapsed >= timeout_)
        {
            std::cout << "[DriveToChargingStation] Timeout reached -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[DriveToChargingStation] Waiting... elapsed=" << elapsed << "s" << std::endl;
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
};

class StatusDriveToChargingDock : public BT::StatefulActionNode
{
public:
    StatusDriveToChargingDock(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          status_(""), timeout_(5.0)
    {
        node_ = rclcpp::Node::make_shared("btStatusDriveToChargingDock");
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                status_ = msg->data;
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override
    {
        getInput("timeout", timeout_);
        start_time_ = std::chrono::steady_clock::now();
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
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                event_ = msg->data;
            });
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout") };
    }

    BT::NodeStatus onStart() override
    {
        getInput("timeout", timeout_);
        start_time_ = std::chrono::steady_clock::now();
        std_msgs::msg::String msg;
        msg.data = "IsRobotCharging";
        pub_->publish(msg);
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
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                last_event_ = msg->data;
            });
    }

        static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("robotLocation") };
    }


    BT::NodeStatus onStart() override
    {
        setOutput("robotLocation", "CHARGING");
        rclcpp::spin_some(node_);
        if (last_event_ == "CHARGING-COMPLETED")
        {
            std::cout << "[IsBatteryFull] CHARGING COMPLETED -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        if (last_event_ == "CHARGING-COMPLETED")
            return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override {
        std::cout << "[IsBatteryFull] HALTED" << std::endl;
    }


private:
    std::string last_event_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
}; 


class BatteryCharged : public TimedCondition
{
public:
    BatteryCharged(const std::string &name, const BT::NodeConfiguration &config)
        : TimedCondition(name, config)
    {
        // Node voor ROS2 publishers
        node_ = rclcpp::Node::make_shared("btBatteryCharged");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    BT::NodeStatus onStart() override
    {
        // Eerste, roep de originele TimedCondition onStart aan om timer te starten
        BT::NodeStatus status = TimedCondition::onStart();

        // Publiceer de naam naar /BehaviorTreeNode
        std_msgs::msg::String msg;
        msg.data = "BatteryCharged";
        pub_->publish(msg);

        return status; // RUNNING
    }

    BT::NodeStatus onRunning() override
    {
        // Roep originele TimedCondition aan
        BT::NodeStatus status = TimedCondition::onRunning();

        // Optioneel: telkens bij tick de status publiceren
        std_msgs::msg::String msg;
        msg.data = "BatteryCharged";
        pub_->publish(msg);

        return status; // RUNNING of SUCCESS als timer afgelopen
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class DriveQuizLocation : public BT::StatefulActionNode
{
public:
    DriveQuizLocation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btDriveQuizLocation");

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
            BT::OutputPort<std::string>("sent_timestamp")
        };
    }

    BT::NodeStatus onStart() override
    {
        // Publish BT node naam
        std_msgs::msg::String bt_msg;
        bt_msg.data = "DriveQuizLocation";
        pub_bt_->publish(bt_msg);

        // Publish coordinate
        sent_coord_.header.stamp = node_->get_clock()->now();
        sent_coord_.header.frame_id = "map";

        double x, y, z;
        getInput("x", x);
        getInput("y", y);
        getInput("z", z);

        sent_coord_.pose.position.x = x;
        sent_coord_.pose.position.y = y;
        sent_coord_.pose.position.z = z;

        sent_coord_.pose.orientation.w = 1.0;
        pub_coord_->publish(sent_coord_);

        // Timestamp opslaan en op blackboard zetten
        sent_timestamp_ = std::to_string(sent_coord_.header.stamp.sec) + "." +
                          std::to_string(sent_coord_.header.stamp.nanosec);
        setOutput("sent_timestamp", sent_timestamp_);

        std::cout << "[DriveQuizLocation] Published coordinate at timestamp: " << sent_timestamp_ << std::endl;

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 5.0;

        start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

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
                    else if (status_code == "05" ||  status_code == "07")
                        //received_failure_ = true;
                        std::cout << "FAILURE ONTVANGEN";
                        
                   
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("sent_timestamp")  // Timestamp uit blackboard
        };
    }

    BT::NodeStatus onStart() override
    {
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

class QuizActive : public BT::StatefulActionNode
{
public:
    QuizActive(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btQuizActive");
        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        std::cout << "[QuizActive] Robot is op locatie, quiz modus actief..." << std::endl;
        std_msgs::msg::String msg;
        msg.data = "QuizActive";
        pub_->publish(msg);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[QuizActive] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};



class BatteryStopDrive : public BT::StatefulActionNode
{
public:
    BatteryStopDrive(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btBatteryStopDrive");

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
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
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    // registreer nodes
    // < > geeft de naam van de c++ node, de (" ... ") geeft de naam van de node in de XML file waar je deze c++ code aan koppelt



    factory.registerNodeType<DriveQuizLocation>("DriveQuizLocation");
    factory.registerNodeType<IsRobotAtQuiz>("IsRobotAtQuiz");

    factory.registerNodeType<BatteryOk>("BatteryOk");
    factory.registerNodeType<DriveToChargingStation>("DriveToChargingStation");
    factory.registerNodeType<StatusDriveToChargingDock>("StatusDriveToChargingDock");
    factory.registerNodeType<IsRobotCharging>("IsRobotCharging");
    factory.registerNodeType<IsBatteryFull>("IsBatteryFull");
    factory.registerNodeType<BatteryCharged>("BatteryCharged");
    factory.registerNodeType<BatteryStopDrive>("BatteryStopDrive");

    factory.registerNodeType<QuizActive>("QuizActive");

    // laad boom uit XML
    auto tree = factory.createTreeFromFile("src/btTestAutocharge/trees/behavior_tree.xml");

    std::cout << "--- Starting BT in continuous mode ---" << std::endl;
    rclcpp::Rate loop_rate(1.0); 

    while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS) {
            std::cout << "--- Tree ticked to SUCCESS ---" << std::endl;
            // Optioneel: reset de boom zodat sommige nodes opnieuw kunnen uitvoeren
            tree.rootNode()->halt();
        }
        else if (status == BT::NodeStatus::FAILURE) {
            std::cout << "--- Tree ticked to FAILURE ---" << std::endl;
            // Optioneel: reset de boom om opnieuw te proberen
            tree.rootNode()->halt();
        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

