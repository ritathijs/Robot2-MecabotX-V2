#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <atomic>
#include <utility>

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

class SetRobotInactive : public BT::SyncActionNode
{
public:
    SetRobotInactive(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config),
          active_pub_(node->create_publisher<std_msgs::msg::Bool>(
              "/RobotActive", 10))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std_msgs::msg::Bool msg;
        msg.data = false;
        active_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
};

class SendDriveWorkArea : public BT::StatefulActionNode
{
public:
    SendDriveWorkArea(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node))
    {
        command_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/robot_command", 10);

        work_area_sub_ =
            node_->create_subscription<std_msgs::msg::Bool>(
                "/IsRobotAtWorkArea",
                10,
                [this](std_msgs::msg::Bool::SharedPtr msg)
                {
                    work_area_reached_ = msg->data;
                });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        work_area_reached_ = false;
        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "DRIVE_TO_WORK_AREA";
        command_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "Navigatie naar de werkzone gestart.");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (work_area_reached_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Werkzone bereikt.");

            return BT::NodeStatus::SUCCESS;
        }

        const auto elapsed =
            std::chrono::steady_clock::now() - start_time_;

        if (elapsed >= timeout_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Timeout bij navigatie naar de werkzone.");

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Navigatieactie naar de werkzone is afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr work_area_sub_;

    bool work_area_reached_{false};

    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds timeout_{120};
};

class SendDriveQuizLocationA : public BT::StatefulActionNode
{
public:
    SendDriveQuizLocationA(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node))
    {
        command_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/robot_command", 10);

        quiz_status_sub_ =
            node_->create_subscription<std_msgs::msg::Bool>(
                "/IsRobotAtQuiz",
                10,
                [this](std_msgs::msg::Bool::SharedPtr msg)
                {
                    quiz_reached_ = msg->data;
                });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        quiz_reached_ = false;
        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "DRIVE_QUIZ_LOCATION_A";
        command_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "Navigatie naar quizlocatie A gestart.");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (quiz_reached_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Quizlocatie bereikt.");

            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() - start_time_ >= timeout_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Timeout bij navigatie naar quizlocatie A.");

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Navigatieactie naar quizlocatie A is afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quiz_status_sub_;

    bool quiz_reached_{false};
    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds timeout_{120};
};

class SendDriveQuizLocationB : public BT::StatefulActionNode
{
public:
    SendDriveQuizLocationB(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node))
    {
        command_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/robot_command", 10);

        quiz_status_sub_ =
            node_->create_subscription<std_msgs::msg::Bool>(
                "/IsRobotAtQuiz",
                10,
                [this](std_msgs::msg::Bool::SharedPtr msg)
                {
                    quiz_reached_ = msg->data;
                });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        quiz_reached_ = false;
        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "DRIVE_QUIZ_LOCATION_B";
        command_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "Navigatie naar quizlocatie B gestart.");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (quiz_reached_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Quizlocatie bereikt.");

            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() - start_time_ >= timeout_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Timeout bij navigatie naar quizlocatie B.");

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Navigatieactie naar quizlocatie B is afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quiz_status_sub_;

    bool quiz_reached_{false};
    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds timeout_{120};
};

class ExecuteActiveQuiz : public BT::StatefulActionNode
{
public:
    ExecuteActiveQuiz(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node))
    {
        command_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/robot_command", 10);

        quiz_finished_sub_ =
            node_->create_subscription<std_msgs::msg::Bool>(
                "/quiz_finished",
                10,
                [this](std_msgs::msg::Bool::SharedPtr msg)
                {
                    if (!msg->data)
                    {
                        // De manager meldt dat de nieuwe quiz is gestart.
                        quiz_started_seen_.store(true);
                    }
                    else if (quiz_started_seen_.load())
                    {
                        // Alleen een voltooiing ná de start telt.
                        quiz_finished_.store(true);
                    }
                });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        quiz_started_seen_.store(false);
        quiz_finished_.store(false);

        std_msgs::msg::String msg;
        msg.data = "START_QUIZ";
        command_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "Quizstart verstuurd; wachten tot de quiz is afgelopen.");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (quiz_finished_.load())
        {
            RCLCPP_INFO(node_->get_logger(), "Quiz is afgelopen.");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Wachten op het quizresultaat is door de Behavior Tree afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr quiz_finished_sub_;

    std::atomic_bool quiz_started_seen_{false};
    std::atomic_bool quiz_finished_{false};
};


class SendDriveToChargingArea : public BT::StatefulActionNode
{
public:
    SendDriveToChargingArea(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(node)
    {
        command_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/robot_command", 10);

        charging_area_sub_ =
            node_->create_subscription<std_msgs::msg::Bool>(
                "/IsRobotNearChargingStation",
                10,
                [this](std_msgs::msg::Bool::SharedPtr msg)
                {
                    if (!msg->data)
                    {
                        saw_not_at_charging_area_ = true;
                    }
                    else if (saw_not_at_charging_area_)
                    {
                        charging_area_reached_ = true;
                    }
                });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        charging_area_reached_ = false;
        saw_not_at_charging_area_ = false;
        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "DRIVE_TO_CHARGING_AREA";
        command_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "Navigatie naar het laadstation-voorpunt gestart.");

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (charging_area_reached_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Laadstation-voorpunt bereikt.");

            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() - start_time_ >= timeout_)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Timeout bij navigatie naar het laadstation-voorpunt.");

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Navigatie naar het laadstation-voorpunt is afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charging_area_sub_;

    bool saw_not_at_charging_area_{false};
    bool charging_area_reached_{false};

    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds timeout_{120};
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

#define DEFINE_WAYPOINT_ACTION(ClassName, Command, StatusTopic)             \
class ClassName : public BT::StatefulActionNode                            \
{                                                                           \
public:                                                                     \
    ClassName(                                                              \
        const std::string& name,                                            \
        const BT::NodeConfiguration& config,                               \
        rclcpp::Node::SharedPtr node)                                       \
        : BT::StatefulActionNode(name, config),                             \
          node_(std::move(node))                                            \
    {                                                                       \
        command_pub_ = node_->create_publisher<std_msgs::msg::String>(      \
            "/robot_command", 10);                                         \
        status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(      \
            StatusTopic, 10,                                                \
            [this](std_msgs::msg::Bool::SharedPtr msg)                      \
            {                                                               \
                reached_ = msg->data;                                      \
            });                                                             \
    }                                                                       \
                                                                            \
    static BT::PortsList providedPorts()                                    \
    {                                                                       \
        return {};                                                          \
    }                                                                       \
                                                                            \
    BT::NodeStatus onStart() override                                       \
    {                                                                       \
        reached_ = false;                                                   \
        start_time_ = std::chrono::steady_clock::now();                     \
                                                                            \
        std_msgs::msg::String msg;                                          \
        msg.data = Command;                                                 \
        command_pub_->publish(msg);                                         \
                                                                            \
        return BT::NodeStatus::RUNNING;                                     \
    }                                                                       \
                                                                            \
    BT::NodeStatus onRunning() override                                     \
    {                                                                       \
        if (reached_)                                                       \
        {                                                                   \
            return BT::NodeStatus::SUCCESS;                                \
        }                                                                   \
                                                                            \
        if (std::chrono::steady_clock::now() - start_time_ >= timeout_)     \
        {                                                                   \
            RCLCPP_ERROR(                                                   \
                node_->get_logger(),                                        \
                "Timeout bij navigatiecommando: %s", Command);             \
            return BT::NodeStatus::FAILURE;                                \
        }                                                                   \
                                                                            \
        return BT::NodeStatus::RUNNING;                                     \
    }                                                                       \
                                                                            \
    void onHalted() override                                                \
    {                                                                       \
        RCLCPP_WARN(                                                        \
            node_->get_logger(),                                           \
            "Waypointactie afgebroken: %s", Command);                       \
    }                                                                       \
                                                                            \
private:                                                                    \
    rclcpp::Node::SharedPtr node_;                                          \
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;       \
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;       \
    bool reached_{false};                                                   \
    std::chrono::steady_clock::time_point start_time_;                     \
    const std::chrono::seconds timeout_{120};                               \
};

DEFINE_WAYPOINT_ACTION(
    SendDriveLocationC, "DRIVE_TO_LOCATION_C", "/IsRobotAtLocationC")
DEFINE_WAYPOINT_ACTION(
    SendDriveLocationD, "DRIVE_TO_LOCATION_D", "/IsRobotAtLocationD")
DEFINE_WAYPOINT_ACTION(
    SendDriveLocationE, "DRIVE_TO_LOCATION_E", "/IsRobotAtLocationE")

DEFINE_CONDITION_NODE(CheckRobotActive, "/IsRobotActive")
DEFINE_CONDITION_NODE(BatteryOk, "/IsBatteryOK")
DEFINE_CONDITION_NODE(IsRobotClearFromDock, "/IsLocalizationValid") 
DEFINE_CONDITION_NODE(IsRobotAtWorkArea, "/IsRobotAtWorkArea")
DEFINE_CONDITION_NODE(WantToStartQuiz, "/WantToStartQuiz") 
DEFINE_CONDITION_NODE(IsRobotAtQuiz, "/IsRobotAtQuiz")
DEFINE_CONDITION_NODE(IsRobotNearChargingStation, "/IsRobotNearChargingStation")
DEFINE_CONDITION_NODE(IsRobotCharging, "/IsRobotCharging")
DEFINE_CONDITION_NODE(IsPersonDetected, "/visitors_nearby")
DEFINE_CONDITION_NODE(IsRobotAtLocationC, "/IsRobotAtLocationC")
DEFINE_CONDITION_NODE(IsRobotAtLocationD, "/IsRobotAtLocationD")
DEFINE_CONDITION_NODE(IsRobotAtLocationE, "/IsRobotAtLocationE")


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
    REGISTER_BT_NODE(SetRobotInactive)

    // Registreer alle Condities
    REGISTER_BT_NODE(CheckRobotActive)
    REGISTER_BT_NODE(BatteryOk)
    REGISTER_BT_NODE(IsRobotClearFromDock)
    REGISTER_BT_NODE(IsRobotAtWorkArea)
    REGISTER_BT_NODE(WantToStartQuiz)
    REGISTER_BT_NODE(IsRobotAtQuiz)
    REGISTER_BT_NODE(IsRobotNearChargingStation)
    REGISTER_BT_NODE(IsRobotCharging)

    REGISTER_BT_NODE(IsPersonDetected)
    REGISTER_BT_NODE(IsRobotAtLocationC)
    REGISTER_BT_NODE(IsRobotAtLocationD)
    REGISTER_BT_NODE(IsRobotAtLocationE)
    REGISTER_BT_NODE(SendDriveLocationC)
    REGISTER_BT_NODE(SendDriveLocationD)
    REGISTER_BT_NODE(SendDriveLocationE)

    // GECORRIGEERD: Universeel en sluitend share-pad naar je XML-boom
    auto tree = factory.createTreeFromFile("/home/wheeltec/wheeltec_ros2/install/mecabot_integration/share/mecabot_integration/config/behavior_tree.xml");

    rclcpp::Rate rate(2); // 10 Hz oftewel elke 100 ms een tick

    while (rclcpp::ok())
    {
        rclcpp::spin_some(bt_node); // Verwerkt alle inkomende topic-callbacks atomair
        tree.tickRoot();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
