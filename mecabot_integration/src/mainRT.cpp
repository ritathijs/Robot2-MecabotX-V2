#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

// =========================================================================
//  1. ACTION NODES (Acties)
// =========================================================================

class ExecuteUndockDrive : public BT::SyncActionNode {
public:
    // GECORRIGEERD: BT::NodeConfig veranderd naar BT::NodeConfiguration
    ExecuteUndockDrive(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/DockingCommand", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "STOP"; 
        pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class SendDriveWorkArea : public BT::SyncActionNode {
public:
    // GECORRIGEERD: BT::NodeConfig veranderd naar BT::NodeConfiguration
    SendDriveWorkArea(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_command", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "DRIVE_TO_WORK_AREA";
        pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class SendDriveQuizLocationA : public BT::SyncActionNode {
public:
    // GECORRIGEERD: BT::NodeConfig veranderd naar BT::NodeConfiguration
    SendDriveQuizLocationA(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_command", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "DRIVE_QUIZ_LOCATION_A"; 
        pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class SendDriveQuizLocationB : public BT::SyncActionNode {
public:
    // GECORRIGEERD: BT::NodeConfig veranderd naar BT::NodeConfiguration
    SendDriveQuizLocationB(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_command", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "DRIVE_QUIZ_LOCATION_B"; 
        pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class ExecuteActiveQuiz : public BT::ActionNodeBase {
public:
    ExecuteActiveQuiz(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_command", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    
    // ActionNodeBase vereist een halt() en een tick()
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "START_QUIZ"; 
        pub_->publish(msg);
        
        // Dit mag nu legaal RUNNING teruggeven zonder crash!
        return BT::NodeStatus::RUNNING; 
    }
    void halt() override {
        // Wordt aangeroepen als de BT deze tak geforceerd afbreekt
        RCLCPP_INFO(node_->get_logger(), "Quiz tak onderbroken.");
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


class SendDriveToChargingArea : public BT::SyncActionNode {
public:
    // GECORRIGEERD: BT::NodeConfig veranderd naar BT::NodeConfiguration
    SendDriveToChargingArea(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/robot_command", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "DRIVE_TO_CHARGING_AREA"; 
        pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class StartInfraroodDocking : public BT::ActionNodeBase {
public:
    StartInfraroodDocking(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::ActionNodeBase(name, config), node_(node) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/DockingCommand", 10);
    }
    static BT::PortsList providedPorts() { return {}; }
    
    BT::NodeStatus tick() override {
        std_msgs::msg::String msg;
        msg.data = "START"; 
        pub_->publish(msg);
        
        // Dit mag nu legaal RUNNING teruggeven zonder crash!
        return BT::NodeStatus::RUNNING; 
    }
    void halt() override {
        RCLCPP_INFO(node_->get_logger(), "Docking procedure onderbroken.");
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

// Dummy of placeholder acties voor de wervingsflow
#define DEFINE_DUMMY_ACTION(ClassName) \
class ClassName : public BT::SyncActionNode { \
public: \
    ClassName(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) \
        : BT::SyncActionNode(name, config), node_(node) {} \
    static BT::PortsList providedPorts() { return {}; } \
    BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; } \
private: rclcpp::Node::SharedPtr node_; \
};

DEFINE_DUMMY_ACTION(RobotExplore)
DEFINE_DUMMY_ACTION(StartDrivingToPeople)
DEFINE_DUMMY_ACTION(CheckingNearbyVisitors)
DEFINE_DUMMY_ACTION(RobotRotationFollowMe)


// =========================================================================
//  2. CONDITION NODES (Voorwaarden gedreven door StatusReader)
// =========================================================================

#define DEFINE_CONDITION_NODE(ClassName, TopicName) \
class ClassName : public BT::ConditionNode { \
public: \
    ClassName(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node) \
        : BT::ConditionNode(name, config), node_(node) { \
        sub_ = node_->create_subscription<std_msgs::msg::Bool>( \
            TopicName, 10, [this](std_msgs::msg::Bool::SharedPtr msg){ last_value_ = msg->data; }); \
    } \
    static BT::PortsList providedPorts() { return {}; } \
    BT::NodeStatus tick() override { \
        return last_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE; \
    } \
private: \
    rclcpp::Node::SharedPtr node_; \
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_; \
    bool last_value_ = false; \
};

DEFINE_CONDITION_NODE(CheckRobotActive, "/IsRobotActive")
DEFINE_CONDITION_NODE(BatteryOk, "/IsBatteryOK")
DEFINE_CONDITION_NODE(IsRobotClearFromDock, "/IsLocalizationValid") 
DEFINE_CONDITION_NODE(IsRobotAtWorkArea, "/IsRobotAtWorkArea")
DEFINE_CONDITION_NODE(WantToStartQuiz, "/WantToStartQuiz") 
DEFINE_CONDITION_NODE(IsRobotAtQuiz, "/IsRobotAtQuiz")
DEFINE_CONDITION_NODE(IsRobotNearChargingStation, "/IsRobotNearChargingStation")
DEFINE_CONDITION_NODE(IsRobotCharging, "/IsRobotCharging")


// =========================================================================
//  3. MAIN EXECUTION
// =========================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto bt_node = std::make_shared<rclcpp::Node>("behavior_tree_node");
    BT::BehaviorTreeFactory factory;

    // Macro builder helper om lambda registraties in te korten
    #define REGISTER_BT_NODE(ClassName) \
    factory.registerBuilder<ClassName>(#ClassName, \
        [bt_node](const std::string& name, const BT::NodeConfiguration& config) { \
            return std::make_unique<ClassName>(name, config, bt_node); \
        });

    // Registreer alle Acties
    REGISTER_BT_NODE(ExecuteUndockDrive)
    REGISTER_BT_NODE(SendDriveWorkArea)
    REGISTER_BT_NODE(SendDriveQuizLocationA)
    REGISTER_BT_NODE(SendDriveQuizLocationB)
    REGISTER_BT_NODE(ExecuteActiveQuiz)
    REGISTER_BT_NODE(SendDriveToChargingArea)
    REGISTER_BT_NODE(StartInfraroodDocking)
    REGISTER_BT_NODE(RobotExplore)
    REGISTER_BT_NODE(StartDrivingToPeople)
    REGISTER_BT_NODE(CheckingNearbyVisitors)
    REGISTER_BT_NODE(RobotRotationFollowMe)

    // Registreer alle Condities
    REGISTER_BT_NODE(CheckRobotActive)
    REGISTER_BT_NODE(BatteryOk)
    REGISTER_BT_NODE(IsRobotClearFromDock)
    REGISTER_BT_NODE(IsRobotAtWorkArea)
    REGISTER_BT_NODE(WantToStartQuiz)
    REGISTER_BT_NODE(IsRobotAtQuiz)
    REGISTER_BT_NODE(IsRobotNearChargingStation)
    REGISTER_BT_NODE(IsRobotCharging)

    // GECORRIGEERD: Universeel en sluitend share-pad naar je XML-boom
    auto tree = factory.createTreeFromFile("/home/wheeltec/wheeltec_ros2/install/mecabot_integration/share/mecabot_integration/config/behavior_tree.xml");

    rclcpp::Rate rate(2); // 10 Hz oftewel elke 100 ms een tick

    while (rclcpp::ok())
    {
        tree.tickRoot();
        rclcpp::spin_some(bt_node); // Verwerkt alle inkomende topic-callbacks atomair
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
