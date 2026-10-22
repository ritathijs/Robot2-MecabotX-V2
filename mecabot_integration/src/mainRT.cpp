#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <atomic>
#include <memory>
#include <utility>

// =========================================================================
//  1. ACTION NODES (Acties)
// =========================================================================

class ExecuteUndockDrive : public BT::StatefulActionNode {
public:
    ExecuteUndockDrive(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node)) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/charge_XSTOP", 10);
        event_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.size() >= std::string("ROBOT-UNDOCKED").size() &&
                    msg->data.compare(
                        msg->data.size() - std::string("ROBOT-UNDOCKED").size(),
                        std::string("ROBOT-UNDOCKED").size(),
                        "ROBOT-UNDOCKED") == 0) {
                    undocked_ = true;
                }
            });
    }
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override {
        undocked_ = false;
        std_msgs::msg::String msg;
        msg.data = "XSTOPS";
        pub_->publish(msg);
        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "Loskoppelen van het laadstation gestart.");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (undocked_) {
            RCLCPP_INFO(node_->get_logger(), "Robot is los van het laadstation.");
            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() - start_time_ >= timeout_) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Timeout: geen bevestiging ontvangen dat de robot los is van het laadstation.");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(), "Loskoppelactie afgebroken.");
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    bool undocked_{false};
    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds timeout_{15};
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

struct AprilTagPatrolState
{
    bool tag_seen_since_cycle_completion{false};
    bool lost{false};
    unsigned int consecutive_cycles_without_tag{0};
};

class CheckAprilTagAtLocationD : public BT::StatefulActionNode
{
public:
    CheckAprilTagAtLocationD(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<AprilTagPatrolState> patrol_state)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node)),
          patrol_state_(std::move(patrol_state))
    {
        reset_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/reset_active", 10);
        result_sub_ =
            node_->create_subscription<std_msgs::msg::String>(
                "/reset_done",
                10,
                [this](std_msgs::msg::String::SharedPtr msg)
                {
                    if (!check_active_) {
                        return;
                    }

                    if (msg->data == "done") {
                        tag_found_ = true;
                        patrol_state_->tag_seen_since_cycle_completion = true;
                    } else if (msg->data == "not_found") {
                        tag_missing_ = true;
                    }
                });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        check_active_ = true;
        tag_found_ = false;
        tag_missing_ = false;
        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "active";
        reset_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "AprilTag-controle bij punt D gestart; maximaal 5 seconden.");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (tag_found_) {
            finish_check();
            RCLCPP_INFO(
                node_->get_logger(),
                "AprilTag bij punt D gevonden; posecorrectie ontvangen.");
            return BT::NodeStatus::SUCCESS;
        }

        if (tag_missing_ ||
            std::chrono::steady_clock::now() - start_time_ >= response_timeout_) {
            finish_check();
            RCLCPP_WARN(
                node_->get_logger(),
                "AprilTag niet gevonden bij punt D; deze ronde telt als "
                "een gemiste controle.");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        finish_check();
        RCLCPP_WARN(
            node_->get_logger(),
            "AprilTag-controle bij punt D afgebroken.");
    }

private:
    void finish_check()
    {
        if (!check_active_) {
            return;
        }

        check_active_ = false;
        std_msgs::msg::String msg;
        msg.data = "inactive";
        reset_pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<AprilTagPatrolState> patrol_state_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reset_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
    bool check_active_{false};
    bool tag_found_{false};
    bool tag_missing_{false};
    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds response_timeout_{6};
};

class CompletePatrolCycle : public BT::StatefulActionNode
{
public:
    CompletePatrolCycle(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<AprilTagPatrolState> patrol_state)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node)),
          patrol_state_(std::move(patrol_state)),
          command_pub_(node_->create_publisher<std_msgs::msg::String>(
              "/robot_command", 10)),
          patrol_reset_pub_(node_->create_publisher<std_msgs::msg::Bool>(
              "/reset_patrol_locations", 10))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        if (patrol_state_->tag_seen_since_cycle_completion) {
            patrol_state_->consecutive_cycles_without_tag = 0;
            RCLCPP_INFO(
                node_->get_logger(),
                "Patrouilleronde voltooid: AprilTag gezien; "
                "teller gemiste rondes teruggezet.");
        } else {
            ++patrol_state_->consecutive_cycles_without_tag;
            RCLCPP_WARN(
                node_->get_logger(),
                "Patrouilleronde voltooid zonder AprilTag (%u/5).",
                patrol_state_->consecutive_cycles_without_tag);
        }
        patrol_state_->tag_seen_since_cycle_completion = false;

        if (patrol_state_->consecutive_cycles_without_tag < 5) {
            std_msgs::msg::Bool reset;
            reset.data = true;
            patrol_reset_pub_->publish(reset);
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_ERROR(
            node_->get_logger(),
            "Robot gestopt: gedurende vijf volledige C-D-E-rondes is geen "
            "AprilTag gezien; verdwaald-status geactiveerd.");

        patrol_state_->lost = true;
        std_msgs::msg::String command;
        command.data = "SHOW_LOST_SCREEN";
        command_pub_->publish(command);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "Afhandeling van de AprilTag-vermistestatus afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<AprilTagPatrolState> patrol_state_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr patrol_reset_pub_;
};

class IsAprilTagLost : public BT::ConditionNode
{
public:
    IsAprilTagLost(
        const std::string& name,
        const BT::NodeConfiguration& config,
        std::shared_ptr<AprilTagPatrolState> patrol_state)
        : BT::ConditionNode(name, config),
          patrol_state_(std::move(patrol_state))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        return patrol_state_->lost
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<AprilTagPatrolState> patrol_state_;
};

class HoldAprilTagLost : public BT::StatefulActionNode
{
public:
    HoldAprilTagLost(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "BT blijft geblokkeerd: AprilTag-vermistestatus actief.");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(
            node_->get_logger(),
            "AprilTag-vermistatus is door een BT-herstart verlaten.");
    }

private:
    rclcpp::Node::SharedPtr node_;
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

class StartInfraroodDocking : public BT::StatefulActionNode {
public:
    StartInfraroodDocking(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(std::move(node)) {
        pub_ = node_->create_publisher<std_msgs::msg::String>("/force_charge", 10);
        charging_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/IsRobotCharging", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                charging_ = msg->data;
            });
        event_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                constexpr char failure_event[] = "DOCKING-FAILED";
                if (msg->data.size() >= sizeof(failure_event) - 1 &&
                    msg->data.compare(
                        msg->data.size() - (sizeof(failure_event) - 1),
                        sizeof(failure_event) - 1,
                        failure_event) == 0) {
                    docking_failed_ = true;
                }
            });
    }
    static BT::PortsList providedPorts() { return {}; }
    
    BT::NodeStatus onStart() override {
        charging_ = false;
        docking_failed_ = false;
        start_time_ = std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "0START";
        pub_->publish(msg);

        RCLCPP_INFO(node_->get_logger(), "IR-docking gestart via /force_charge.");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (charging_) {
            RCLCPP_INFO(node_->get_logger(), "Laden bevestigd; IR-docking geslaagd.");
            return BT::NodeStatus::SUCCESS;
        }

        if (docking_failed_) {
            RCLCPP_ERROR(node_->get_logger(), "IR-docking is mislukt.");
            return BT::NodeStatus::FAILURE;
        }

        if (std::chrono::steady_clock::now() - start_time_ >= timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Timeout tijdens IR-docking.");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(), "IR-dockingactie afgebroken.");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr charging_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    bool charging_{false};
    bool docking_failed_{false};
    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds timeout_{200};
};

class VisitorTargetMonitor
{
public:
    explicit VisitorTargetMonitor(rclcpp::Node::SharedPtr node)
    {
        target_distance_sub_ =
            node->create_subscription<std_msgs::msg::Float32>(
                "/target_distance",
                10,
                [this](std_msgs::msg::Float32::SharedPtr msg)
                {
                    latest_distance_ = msg->data;
                    last_update_ = std::chrono::steady_clock::now();
                    has_update_ = true;

                    if (ignore_current_person_) {
                        if (latest_distance_ <= 0.0F) {
                            ++clear_samples_;
                            if (clear_samples_ >= required_clear_samples_) {
                                ignore_current_person_ = false;
                                clear_samples_ = 0;
                            }
                        } else {
                            clear_samples_ = 0;
                        }
                    }
                });
    }

    bool has_fresh_sample() const
    {
        return has_update_ &&
            std::chrono::steady_clock::now() - last_update_ <= sample_timeout_;
    }

    bool person_detected() const
    {
        return has_fresh_sample() &&
            latest_distance_ > 0.0F &&
            !ignore_current_person_;
    }

    float distance() const
    {
        return latest_distance_;
    }

    void ignore_current_person()
    {
        ignore_current_person_ = true;
        clear_samples_ = 0;
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_distance_sub_;
    float latest_distance_{0.0F};
    bool has_update_{false};
    bool ignore_current_person_{false};
    unsigned int clear_samples_{0};
    const unsigned int required_clear_samples_{2};
    std::chrono::steady_clock::time_point last_update_{};
    const std::chrono::seconds sample_timeout_{3};
};

class IsPersonDetected : public BT::ConditionNode
{
public:
    IsPersonDetected(
        const std::string& name,
        const BT::NodeConfiguration& config,
        std::shared_ptr<VisitorTargetMonitor> target_monitor)
        : BT::ConditionNode(name, config),
          target_monitor_(std::move(target_monitor))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        return target_monitor_->person_detected()
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<VisitorTargetMonitor> target_monitor_;
};

class StartDrivingToPeople : public BT::SyncActionNode
{
public:
    StartDrivingToPeople(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config),
          screen_pub_(node->create_publisher<std_msgs::msg::String>(
              "/screen_command", 10))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        std_msgs::msg::String screen;
        screen.data = "robot-go-to-visitors";
        screen_pub_->publish(screen);
        RCLCPP_INFO(
            rclcpp::get_logger("behavior_tree_node"),
            "Bezoeker gedetecteerd; de robot nadert de persoon.");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr screen_pub_;
};

class CheckingNearbyVisitors : public BT::StatefulActionNode
{
public:
    CheckingNearbyVisitors(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<VisitorTargetMonitor> target_monitor)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node)),
          target_monitor_(std::move(target_monitor)),
          behavior_node_pub_(node_->create_publisher<std_msgs::msg::String>(
              "/BehaviorTreeNode", 10)),
          screen_pub_(node_->create_publisher<std_msgs::msg::String>(
              "/screen_command", 10))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        start_time_ = std::chrono::steady_clock::now();
        last_person_seen_ = start_time_;
        following_person_ = false;

        publish_behavior_node("robot-arrived-at-visitors");
        RCLCPP_INFO(
            node_->get_logger(),
            "Navigatie naar de bezoeker gestart; doelafstand is maximaal 1,8 m.");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        const auto now = std::chrono::steady_clock::now();
        if (target_monitor_->person_detected()) {
            last_person_seen_ = now;

            if (target_monitor_->distance() <= stop_distance_) {
                publish_behavior_node("robot-arrived-at-visitors");
                publish_screen("robot-arrived-at-visitors");
                following_person_ = false;
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Bezoeker bereikt; navigatie gestopt op circa 1,8 m.");
                return BT::NodeStatus::SUCCESS;
            }

            if (!following_person_) {
                publish_behavior_node("RobotRotationFollowMe");
                following_person_ = true;
            }
        } else if (
            following_person_ &&
            now - last_person_seen_ >= target_lost_timeout_) {
            publish_behavior_node("robot-arrived-at-visitors");
            publish_screen("robot-explore");
            RCLCPP_WARN(
                node_->get_logger(),
                "Bezoeker niet langer zichtbaar tijdens naderen; patrouille hervat.");
            return BT::NodeStatus::FAILURE;
        } else if (
            !following_person_ &&
            now - start_time_ >= initial_detection_timeout_) {
            RCLCPP_WARN(
                node_->get_logger(),
                "Geen actuele bezoekerpositie ontvangen; patrouille hervat.");
            return BT::NodeStatus::FAILURE;
        }

        if (now - start_time_ >= approach_timeout_) {
            publish_behavior_node("robot-arrived-at-visitors");
            RCLCPP_WARN(
                node_->get_logger(),
                "Timeout bij het naderen van de bezoeker; patrouille hervat.");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        publish_behavior_node("robot-arrived-at-visitors");
        RCLCPP_WARN(
            node_->get_logger(),
            "Bezoekersnadering onderbroken.");
    }

private:
    void publish_behavior_node(const std::string& name)
    {
        std_msgs::msg::String msg;
        msg.data = name;
        behavior_node_pub_->publish(msg);
    }

    void publish_screen(const std::string& name)
    {
        std_msgs::msg::String msg;
        msg.data = name;
        screen_pub_->publish(msg);
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<VisitorTargetMonitor> target_monitor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_node_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr screen_pub_;
    bool following_person_{false};
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_person_seen_;
    const float stop_distance_{1.8F};
    const std::chrono::seconds initial_detection_timeout_{5};
    const std::chrono::seconds target_lost_timeout_{3};
    const std::chrono::seconds approach_timeout_{120};
};

class RobotRotationFollowMe : public BT::StatefulActionNode
{
public:
    RobotRotationFollowMe(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<VisitorTargetMonitor> target_monitor)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node)),
          target_monitor_(std::move(target_monitor)),
          no_response_sub_(node_->create_subscription<std_msgs::msg::Bool>(
              "/visitor_no_response",
              10,
              [this](std_msgs::msg::Bool::SharedPtr msg)
              {
                  no_response_received_ = msg->data;
              }))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        no_response_received_ = false;
        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(
            node_->get_logger(),
            "Wachten op reactie van de bezoeker (maximaal 25 seconden).");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        if (no_response_received_) {
            target_monitor_->ignore_current_person();
            RCLCPP_INFO(
                node_->get_logger(),
                "Keuzescherm verlopen; bezoeker wordt losgelaten en de zoekrotatie start.");
            return BT::NodeStatus::SUCCESS;
        }

        if (std::chrono::steady_clock::now() - start_time_ < response_timeout_) {
            return BT::NodeStatus::RUNNING;
        }

        target_monitor_->ignore_current_person();
        RCLCPP_INFO(
            node_->get_logger(),
            "Geen START ontvangen binnen 25 seconden; bezoeker wordt tijdelijk "
            "genegeerd tijdens de rotatie.");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Wachten op bezoekersreactie onderbroken door een hogere BT-prioriteit.");
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<VisitorTargetMonitor> target_monitor_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr no_response_sub_;
    bool no_response_received_{false};
    std::chrono::steady_clock::time_point start_time_;
    const std::chrono::seconds response_timeout_{25};
};

class VisitorNoResponseRecovery : public BT::StatefulActionNode
{
public:
    VisitorNoResponseRecovery(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<VisitorTargetMonitor> target_monitor)
        : BT::StatefulActionNode(name, config),
          node_(std::move(node)),
          target_monitor_(std::move(target_monitor)),
          behavior_node_pub_(node_->create_publisher<std_msgs::msg::String>(
              "/BehaviorTreeNode", 10)),
          tracking_pause_pub_(node_->create_publisher<std_msgs::msg::Bool>(
              "/visitor_tracking_paused", 10)),
          turn_pub_(node_->create_publisher<geometry_msgs::msg::Twist>(
              "/turn_cmd_vel", 10)),
          search_pub_(node_->create_publisher<geometry_msgs::msg::Twist>(
              "/search_cmd_vel", 10)),
          screen_pub_(node_->create_publisher<std_msgs::msg::String>(
              "/screen_command", 10))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        phase_ = Phase::QuarterTurn;
        phase_start_ = std::chrono::steady_clock::now();
        target_monitor_->ignore_current_person();
        publish_behavior_node("robot-arrived-at-visitors");
        publish_tracking_pause(true);

        RCLCPP_INFO(
            node_->get_logger(),
            "Bezoeker reageerde niet: tracking gepauzeerd voor een kwartslag "
            "van circa 90 graden.");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        const auto now = std::chrono::steady_clock::now();
        if (phase_ == Phase::QuarterTurn) {
            if (now - phase_start_ < quarter_turn_duration_) {
                publish_angular_velocity(turn_pub_, quarter_turn_speed_);
                return BT::NodeStatus::RUNNING;
            }

            publish_stop(turn_pub_);
            publish_tracking_pause(false);
            phase_ = Phase::VisitorScan;
            phase_start_ = now;
            RCLCPP_INFO(
                node_->get_logger(),
                "Kwartslag voltooid; tracking actief tijdens verdere zoekrotatie.");
        }

        if (target_monitor_->person_detected()) {
            publish_stop(search_pub_);
            publish_screen("robot-go-to-visitors");
            RCLCPP_INFO(
                node_->get_logger(),
                "Nieuwe bezoeker gezien tijdens de zoekrotatie; robot gaat "
                "opnieuw naderen.");
            return BT::NodeStatus::SUCCESS;
        }

        if (now - phase_start_ < visitor_scan_duration_) {
            publish_angular_velocity(search_pub_, visitor_scan_speed_);
            return BT::NodeStatus::RUNNING;
        }

        publish_stop(search_pub_);
        publish_screen("robot-explore");
        RCLCPP_INFO(
            node_->get_logger(),
            "Zoekrotatie voltooid zonder nieuwe bezoeker; patrouille wordt hervat.");
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        publish_stop(turn_pub_);
        publish_stop(search_pub_);
        publish_tracking_pause(false);
        RCLCPP_WARN(
            node_->get_logger(),
            "Bezoekers-herstelrotatie onderbroken.");
    }

private:
    enum class Phase
    {
        QuarterTurn,
        VisitorScan
    };

    void publish_behavior_node(const std::string& name)
    {
        std_msgs::msg::String msg;
        msg.data = name;
        behavior_node_pub_->publish(msg);
    }

    void publish_tracking_pause(bool paused)
    {
        std_msgs::msg::Bool msg;
        msg.data = paused;
        tracking_pause_pub_->publish(msg);
    }

    void publish_screen(const std::string& name)
    {
        std_msgs::msg::String msg;
        msg.data = name;
        screen_pub_->publish(msg);
    }

    static void publish_angular_velocity(
        const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher,
        double angular_velocity)
    {
        geometry_msgs::msg::Twist msg;
        msg.angular.z = angular_velocity;
        publisher->publish(msg);
    }

    static void publish_stop(
        const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& publisher)
    {
        publisher->publish(geometry_msgs::msg::Twist());
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<VisitorTargetMonitor> target_monitor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_node_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_pause_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turn_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr search_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr screen_pub_;
    Phase phase_{Phase::QuarterTurn};
    std::chrono::steady_clock::time_point phase_start_;
    const double quarter_turn_speed_{0.3};
    const std::chrono::seconds quarter_turn_duration_{5};
    const double visitor_scan_speed_{0.25};
    const std::chrono::seconds visitor_scan_duration_{38};
};


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

class RobotStateReady : public BT::ConditionNode
{
public:
    RobotStateReady(
        const std::string& name,
        const BT::NodeConfiguration& config,
        rclcpp::Node::SharedPtr node)
        : BT::ConditionNode(name, config),
          node_(std::move(node))
    {
        auto qos = rclcpp::QoS(1).transient_local();
        status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/IsRobotStateReady",
            qos,
            [this](std_msgs::msg::Bool::SharedPtr msg)
            {
                ready_ = msg->data;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        return ready_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr status_sub_;
    bool ready_{false};
};

DEFINE_CONDITION_NODE(CheckRobotActive, "/IsRobotActive")
DEFINE_CONDITION_NODE(BatteryOk, "/IsBatteryOK")
DEFINE_CONDITION_NODE(IsRobotDocked, "/IsRobotDocked")
DEFINE_CONDITION_NODE(IsRobotAtWorkArea, "/IsRobotAtWorkArea")
DEFINE_CONDITION_NODE(WantToStartQuiz, "/WantToStartQuiz") 
DEFINE_CONDITION_NODE(IsRobotAtQuiz, "/IsRobotAtQuiz")
DEFINE_CONDITION_NODE(IsRobotNearChargingStation, "/IsRobotNearChargingStation")
DEFINE_CONDITION_NODE(IsRobotCharging, "/IsRobotCharging")
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

    auto april_tag_patrol_state = std::make_shared<AprilTagPatrolState>();
    auto visitor_target_monitor =
        std::make_shared<VisitorTargetMonitor>(bt_node);

    // Registreer alle Acties
    REGISTER_BT_NODE(ExecuteUndockDrive)
    REGISTER_BT_NODE(SendDriveWorkArea)
    REGISTER_BT_NODE(SendDriveQuizLocationA)
    REGISTER_BT_NODE(SendDriveQuizLocationB)
    REGISTER_BT_NODE(ExecuteActiveQuiz)
    REGISTER_BT_NODE(SendDriveToChargingArea)
    REGISTER_BT_NODE(StartInfraroodDocking)
    REGISTER_BT_NODE(SetRobotInactive)
    REGISTER_BT_NODE(HoldAprilTagLost)
    factory.registerBuilder<StartDrivingToPeople>(
        "StartDrivingToPeople",
        [bt_node](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<StartDrivingToPeople>(
                name, config, bt_node);
        });
    factory.registerBuilder<CheckingNearbyVisitors>(
        "CheckingNearbyVisitors",
        [bt_node, visitor_target_monitor](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<CheckingNearbyVisitors>(
                name, config, bt_node, visitor_target_monitor);
        });
    factory.registerBuilder<RobotRotationFollowMe>(
        "RobotRotationFollowMe",
        [bt_node, visitor_target_monitor](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<RobotRotationFollowMe>(
                name, config, bt_node, visitor_target_monitor);
        });
    factory.registerBuilder<VisitorNoResponseRecovery>(
        "VisitorNoResponseRecovery",
        [bt_node, visitor_target_monitor](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<VisitorNoResponseRecovery>(
                name, config, bt_node, visitor_target_monitor);
        });
    factory.registerBuilder<CheckAprilTagAtLocationD>(
        "CheckAprilTagAtLocationD",
        [bt_node, april_tag_patrol_state](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<CheckAprilTagAtLocationD>(
                name, config, bt_node, april_tag_patrol_state);
        });
    factory.registerBuilder<CompletePatrolCycle>(
        "CompletePatrolCycle",
        [bt_node, april_tag_patrol_state](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<CompletePatrolCycle>(
                name, config, bt_node, april_tag_patrol_state);
        });
    factory.registerBuilder<IsAprilTagLost>(
        "IsAprilTagLost",
        [april_tag_patrol_state](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<IsAprilTagLost>(
                name, config, april_tag_patrol_state);
        });

    // Registreer alle Condities
    REGISTER_BT_NODE(CheckRobotActive)
    REGISTER_BT_NODE(BatteryOk)
    REGISTER_BT_NODE(IsRobotDocked)
    REGISTER_BT_NODE(RobotStateReady)
    REGISTER_BT_NODE(IsRobotAtWorkArea)
    REGISTER_BT_NODE(WantToStartQuiz)
    REGISTER_BT_NODE(IsRobotAtQuiz)
    REGISTER_BT_NODE(IsRobotNearChargingStation)
    REGISTER_BT_NODE(IsRobotCharging)

    REGISTER_BT_NODE(IsRobotAtLocationC)
    REGISTER_BT_NODE(IsRobotAtLocationD)
    REGISTER_BT_NODE(IsRobotAtLocationE)
    REGISTER_BT_NODE(SendDriveLocationC)
    REGISTER_BT_NODE(SendDriveLocationD)
    REGISTER_BT_NODE(SendDriveLocationE)
    factory.registerBuilder<IsPersonDetected>(
        "IsPersonDetected",
        [visitor_target_monitor](
            const std::string& name,
            const BT::NodeConfiguration& config) {
            return std::make_unique<IsPersonDetected>(
                name, config, visitor_target_monitor);
        });

    const auto behavior_tree_path =
        ament_index_cpp::get_package_share_directory("mecabot_integration") +
        "/config/behavior_tree.xml";
    auto tree = factory.createTreeFromFile(behavior_tree_path);

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
