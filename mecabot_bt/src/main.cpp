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

#include "httplib.h"         // Lokaal gedownload in je map (gebruikt " ")

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
using namespace std::chrono_literals;


#include <nlohmann/json.hpp>

using json = nlohmann::json;

bool updateRobotStatus(const json& fieldsToUpdate)
{
    httplib::Client cli("http://10.0.0.11");

    auto res = cli.Post(
        "/robot-status/insert-robot-status",
        fieldsToUpdate.dump(),
        "application/json");

    if (res && res->status == 200)
    {
        try
        {
            json response = json::parse(res->body);

            if (response.contains("succes") && response["succes"] == true)
            {
                return true;
            }
        }
        catch (const json::parse_error& e)
        {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
        }
    }

    if (res)
    {
        std::cout << "Status: " << res->status << std::endl;
        std::cout << "Body: " << res->body << std::endl;
    }

    std::string err = res ? std::to_string(res->status) : "Connection error";
    std::cerr << "Update robot status failed: " << err << std::endl;

    return false;
}


// Deze functie accepteert een lijst met veldnamen en geeft een JSON-object terug
json retrieveRobotStatus(const std::vector<std::string>& fields) {
    json result = json::object(); // Maak standaard een leeg JSON-object aan

    // 1. Bouw de komma-gescheiden string op van de gevraagde velden
    std::string fieldsQuery = "";
    for (size_t i = 0; i < fields.size(); ++i) {
        fieldsQuery += fields[i];
        if (i < fields.size() - 1) {
            fieldsQuery += ",";
        }
    }

    // 2. Maak de client aan
    httplib::Client cli("http://10.0.0.11");

    // 3. Bouw de exacte URL met de dynamische query
    std::string path = "/robot-status/get-robot-status";
    if (!fieldsQuery.empty()) {
        path += "?fields=" + fieldsQuery;
    }

    // 4. Doe het GET-verzoek
    auto res = cli.Get(path.c_str());

    if (res && res->status == 200) {
        try {
            json responseObj = json::parse(res->body);

            // 5. Controleer of de API succes meldt en of er een 'data' object is
            if (responseObj.contains("succes") && responseObj["succes"] == true &&
                responseObj.contains("data")) {
                
                // Geef de inhoud van "data" terug (hier zitten jouw gevraagde velden in)
                result = responseObj["data"];
            }
        } catch (const json::parse_error& e) {
            std::cerr << "Fout bij het parsen van de JSON: " << e.what() << std::endl;
        }
    } else {
        std::string errorMsg = res ? std::to_string(res->status) : "Netwerk/Verbindingsfout";
        std::cerr << "Fout bij het ophalen van instellingen: HTTP " << errorMsg << std::endl;
    }

    return result;
}



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
// Node die altijd draait (bovenaan in BT)

class CheckNetworkError : public BT::StatefulActionNode
{
public:
    CheckNetworkError(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btCheckNetworkError");

        rclcpp::QoS qos(1);
        qos.reliable();

        // -----------------------------
        // BESTAANDE subscriber behouden
        // -----------------------------
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "quizbtnode_activestatus",
            qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                last_message_ = msg->data;
            });

        // -----------------------------
        // NIEUWE subscriber
        // -----------------------------
        ask_button_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "ask_button_quiz",
            qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                last_button_message_ = msg->data;
            });

        // Publisher naar rpitopic
        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override
    {
        last_message_.clear();
        last_button_message_.clear();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        // ==========================================================
        // BESTAANDE functionaliteit behouden
        // ==========================================================
        if (last_message_ == "ask-is-active")
        {
            last_message_.clear();

            json robotStatus = retrieveRobotStatus({"robotActive"});

            bool robotActive = false;

            if (robotStatus.contains("robotActive"))
            {
                robotActive = robotStatus["robotActive"].get<bool>();
            }

            std_msgs::msg::String msg;

            if (robotActive)
            {
                std::cout << "[CheckNetworkError] robotActive = true" << std::endl;
                msg.data = "RobotIsActiveTrue";
            }
            else
            {
                std::cout << "[CheckNetworkError] robotActive = false" << std::endl;
                msg.data = "RobotIsActiveFalse";
            }

            rpi_pub_->publish(msg);
        }

        // ==========================================================
        // NIEUWE functionaliteit
        // ==========================================================

        if (last_button_message_ == "robot-activeButtonToggled-false")
        {
            last_button_message_.clear();

            std::cout << "[CheckNetworkError] Active button -> FALSE" << std::endl;

            // Database op false zetten
            updateRobotStatus({
                {"robotActive", false}
            });

            // Bevestiging terugsturen
            std_msgs::msg::String msg;
            msg.data = "RobotIsActiveFalse";
            rpi_pub_->publish(msg);
        }
        else if (last_button_message_ == "robot-activeButtonToggled-true")
        {
            last_button_message_.clear();

            std::cout << "[CheckNetworkError] Active button -> TRUE" << std::endl;

            std_msgs::msg::String msg;

            if (batteryLowInFile())
            {
                std::cout << "[CheckNetworkError] Battery LOW -> robot blijft inactive" << std::endl;

                // Database terug op false
                updateRobotStatus({
                    {"robotActive", false}
                });

                msg.data = "RobotIsActiveFalse";
            }
            else
            {
                std::cout << "[CheckNetworkError] Battery OK -> robot wordt active" << std::endl;

                // Database op true
                updateRobotStatus({
                    {"robotActive", true}
                });

                msg.data = "RobotIsActiveTrue";
            }

            rpi_pub_->publish(msg);
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[CheckNetworkError] HALTED" << std::endl;
    }

private:

    bool batteryLowInFile()
    {
        const std::string filePath =
            "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt";

        std::ifstream file(filePath);

        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line == "BATTERY-LOW")
            {
                return true;
            }
        }

        return false;
    }

private:
    rclcpp::Node::SharedPtr node_;

    // Bestaande subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // Nieuwe subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ask_button_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;

    // Bestaande variabele
    std::string last_message_;

    // Nieuwe variabele
    std::string last_button_message_;

    // Mag blijven staan indien elders gebruikt
    std::string last_event_;
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

        setOutput("chargingInteger", 0);
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
            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        if (last_event_ == "BATTERY-LOW")
        {
            std::cout << "[BatteryOk] BATTERY LOW via bericht on start" << std::endl;
            updateRobotStatus({
                {"robotActive", false}
            });

            setOutput("chargingInteger", 0);  // BATTERY-LOW bericht komt altijd met 0 voor (want telkens nieuwe sessie)
            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

            if (batteryLowInFile())
            {
                std::cout << "[BatteryOk] BATTERY-LOW gevonden in bestand. on start" << std::endl;
                updateRobotStatus({
                    {"robotActive", false}
                });


                updateSkipDrive();
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


            std::cout << "[BatteryOk] FORCE-CHARGING detected in on running -> sending START" << std::endl;


            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        if (last_event_ == "BATTERY-LOW")
        {
            std::cout << "[BatteryOk] BAT LOW via bericht" << std::endl;

            int getal = updateChargingCounter();
            updateSkipDrive();  // check bat_admin_status
            return BT::NodeStatus::FAILURE;
        }

        if (batteryLowInFile())
        {
            std::cout << "[BatteryOk] BATTERY-LOW gevonden in bestand on run." << std::endl;
            updateSkipDrive();
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}


private:

    bool batteryLowInFile()
    {

        const std::string filePath = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt";
        std::ifstream file(filePath);

        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line == "BATTERY-LOW")
            {
                return true;
            }
        }

        return false;
    }

    std::string last_event_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
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
    CheckInWorkingZone(const std::string& name,
                       const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btInWorkingZone");

        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("robotLocation"),

            // Blackboard output
            BT::OutputPort<std::string>("robotLocationBAT"),

            BT::InputPort<bool>("skip_drivetoworkarea"),

            // Nieuwe input
            BT::InputPort<bool>("show_verdwaald")
        };
    }

    BT::NodeStatus tick() override
    {
        // =====================================================
        // Check show_verdwaald blackboard
        // =====================================================
        bool show_verdwaald = false;

        getInput("show_verdwaald", show_verdwaald);

        if (show_verdwaald)
        {
            std::cout
                << "[CheckInWorkingZone] show_verdwaald = TRUE -> checking status file"
                << std::endl;

            const std::string status_file =
                "/home/wheeltec/wheeltec_ros2/src/april_tabloo/status.txt";

            std::ifstream file(status_file);

            if (file.is_open())
            {
                std::string status;
                std::getline(file, status);
                file.close();

                // verwijder eventuele spaties/newline
                status.erase(
                    std::remove_if(status.begin(),
                                   status.end(),
                                   [](unsigned char c)
                                   {
                                       return std::isspace(c);
                                   }),
                    status.end());

                std::cout
                    << "[CheckInWorkingZone] status.txt = "
                    << status
                    << std::endl;


                if (status == "NOK")
                {
                    std::cout
                        << "[CheckInWorkingZone] Status NOK -> FORCE SUCCESS"
                        << std::endl;

                    return BT::NodeStatus::SUCCESS;
                }

                if (status == "OK")
                {
                    std::cout
                        << "[CheckInWorkingZone] Status OK -> normale werking"
                        << std::endl;
                }
            }
            else
            {
                std::cout
                    << "[CheckInWorkingZone] Kan status file niet openen -> normale werking"
                    << std::endl;
            }
        }


        // =====================================================
        // Bestaande logica
        // =====================================================

        std::time_t now = std::time(nullptr);
        std::tm *local = std::localtime(&now);

        char day_codes[] = {'U', 'M', 'D', 'W', 'T', 'F', 'S'};
        char current_day_code = day_codes[local->tm_wday];

        int current_time_val =
            (local->tm_hour * 100) + local->tm_min;


        bool is_working_time = true;


        std_msgs::msg::String bt_msg;
        std_msgs::msg::String rpi_msg;


        std::cout
            << "[CheckInWorkingZone] Een robot werkt altijd!"
            << std::endl;


        setOutput("robotLocationBAT",
                  std::string("WORKING"));


        bt_msg.data = "CheckInWorkingZone-WORKING";
        pub_->publish(bt_msg);


        rpi_msg.data = "RobotStarting";
        rpi_pub_->publish(rpi_msg);



        bool skip_drive_to_workarea = false;

        getInput("skip_drivetoworkarea",
                 skip_drive_to_workarea);



        std::string location;

        if (!getInput("robotLocation", location))
        {
            std::cerr
                << "[CheckInWorkingZone] Geen robotLocation!"
                << std::endl;


            if (skip_drive_to_workarea)
            {
                std::cout
                    << "[CheckInWorkingZone] skip_drivetoworkarea = TRUE -> FORCE SUCCESS"
                    << std::endl;

                return BT::NodeStatus::SUCCESS;
            }


            return BT::NodeStatus::FAILURE;
        }



        // AL IN WORKING AREA
        if (location == "WORKING")
        {
            return BT::NodeStatus::SUCCESS;
        }



        if (skip_drive_to_workarea)
        {
            std::cout
                << "[CheckInWorkingZone] skip_drivetoworkarea = TRUE -> FORCE SUCCESS"
                << std::endl;

            return BT::NodeStatus::SUCCESS;
        }



        return BT::NodeStatus::FAILURE;
    }


private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;
};


// BT node die een doelpositie naar charging station stuurt via PoseStamped
// BT node die een doelpositie naar charging station stuurt via PoseStamped
class RobotDriveToChargingStation : public BT::StatefulActionNode
{
public:
    RobotDriveToChargingStation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btRobotDriveToChargingStation");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
        pub_coord_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/btDriveCoord", 10);

        // Raspberry Pi topic
        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<bool>("skip_drive2charging"),
            BT::OutputPort<std::string>("charging_sent_timestamp"),
            BT::OutputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("connection_chargeStatus"),
            BT::InputPort<bool>("skip_robotdrivechargingstation"),
            BT::OutputPort<bool>("skip_drivetoworkarea")

        };
    }

    BT::NodeStatus onStart() override
    {
        setOutput("skip_drivetoworkarea", false);

        // Eerst controleren of rijden moet worden overgeslagen
        bool skip = false;
        getInput("skip_drive2charging", skip);

        if (skip)
        {
            std::cout << "[RobotDriveToChargingStation] skip wegens robot al tegen laadstation" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    
        bool skip_robot_drive = false;
        if (getInput("skip_robotdrivechargingstation", skip_robot_drive))
        {
            if (skip_robot_drive)
            {
                std::cout << "[RobotDriveToChargingStation] skip wegens manual drive naar station" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }

        std_msgs::msg::String bt_msg;
        bt_msg.data = "RobotDriveToChargingStation";
        pub_bt_->publish(bt_msg);

        // Raspberry Pi informeren
        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotGoCharge";
        pub_quiz_->publish(quiz_msg);

        // Blackboard variabele zetten
        setOutput("bat_admin_status", std::string("STOP"));

        setOutput("connection_chargeStatus", true); // Je moet niet kijken naar disconnect wanneer robot in laadcyclus is

        // ===============================
        // Charger positie uit JSON lezen
        // ===============================
        std::string filePath =
            "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json";

        std::ifstream file(filePath);

        if (!file.is_open())
        {
            std::cerr << "[RobotDriveToChargingStation] Kan JSON niet openen: "
                      << filePath << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        json j;

        try
        {
            file >> j;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[RobotDriveToChargingStation] JSON parse fout: "
                      << e.what() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Pose opbouwen
        sent_coord_.header.stamp = node_->get_clock()->now();
        sent_coord_.header.frame_id = "map";

        sent_coord_.pose.position.x = j.at("p_x").get<double>();
        sent_coord_.pose.position.y = j.at("p_y").get<double>();
        sent_coord_.pose.position.z = 0.0;

        sent_coord_.pose.orientation.x = 0.0;
        sent_coord_.pose.orientation.y = 0.0;
        sent_coord_.pose.orientation.z = j.at("orien_z").get<double>();
        sent_coord_.pose.orientation.w = j.at("orien_w").get<double>();

        while (pub_coord_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Waiting for subscribers on /btDriveCoord...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        pub_coord_->publish(sent_coord_);

        // Timestamp bewaren
        sent_timestamp_ =
            std::to_string(sent_coord_.header.stamp.sec) + "." +
            std::to_string(sent_coord_.header.stamp.nanosec);

        setOutput("charging_sent_timestamp", sent_timestamp_);

        std::cout << "[RobotDriveToChargingStation] Published charger coordinate from JSON at timestamp: "
                  << sent_timestamp_ << std::endl;

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 5.0;

        start_time_ = std::chrono::steady_clock::now();

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start_time_)
                           .count();

        if (elapsed >= timeout_)
        {
            std::cout << "[RobotDriveToChargingStation] Timeout ("
                      << timeout_ << "s) -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[RobotDriveToChargingStation] HALTED" << std::endl;
    }

private:
    double timeout_;
    std::string sent_timestamp_;
    std::chrono::steady_clock::time_point start_time_;
    geometry_msgs::msg::PoseStamped sent_coord_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_coord_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
};
class FallbackDriveToChargingStation : public BT::StatefulActionNode
{
public:

    FallbackDriveToChargingStation(const std::string &name,
                                   const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btFallbackDriveToChargingStation");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_coord_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/btDriveCoord", 10);

        // Raspberry Pi topic
        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }


    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),

            BT::OutputPort<std::string>("charging_sent_timestamp"),
            BT::OutputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("connection_chargeStatus")
        };
    }


    BT::NodeStatus onStart() override
    {

        std_msgs::msg::String bt_msg;
        bt_msg.data = "FallbackDriveToChargingStation";
        pub_bt_->publish(bt_msg);


        // Raspberry Pi informeren
        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotGoCharge";
        pub_quiz_->publish(quiz_msg);


        // Blackboard statussen
        setOutput("bat_admin_status", std::string("STOP"));

        // Niet controleren op disconnect tijdens laadcyclus
        setOutput("connection_chargeStatus", true);



                // ===============================
                // Pose uit BT XML lezen
                // ===============================
        // ===============================
        // Charger positie uit JSON lezen
        // ===============================

        std::string filePath =
            "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json";

        std::ifstream file(filePath);

        if (!file.is_open())
        {
            std::cerr
                << "[FallbackDriveToChargingStation] Kan JSON niet openen: "
                << filePath
                << std::endl;

            return BT::NodeStatus::FAILURE;
        }

        json j;

        try
        {
            file >> j;
        }
        catch (const std::exception &e)
        {
            std::cerr
                << "[FallbackDriveToChargingStation] JSON parse fout: "
                << e.what()
                << std::endl;

            return BT::NodeStatus::FAILURE;
        }

        sent_coord_.header.stamp = node_->get_clock()->now();
        sent_coord_.header.frame_id = "map";

        sent_coord_.pose.position.x = j.at("p_x").get<double>();
        sent_coord_.pose.position.y = j.at("p_y").get<double>();
        sent_coord_.pose.position.z = 0.0;

        sent_coord_.pose.orientation.x = 0.0;
        sent_coord_.pose.orientation.y = 0.0;
        sent_coord_.pose.orientation.z = j.at("orien_z").get<double>();
        sent_coord_.pose.orientation.w = j.at("orien_w").get<double>();



        while (pub_coord_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Waiting for subscribers on /btDriveCoord...");

            rclcpp::sleep_for(
                std::chrono::milliseconds(100));
        }



        pub_coord_->publish(sent_coord_);



        sent_timestamp_ =
            std::to_string(sent_coord_.header.stamp.sec)
            + "."
            +
            std::to_string(sent_coord_.header.stamp.nanosec);



        setOutput(
            "charging_sent_timestamp",
            sent_timestamp_);



        std::cout
            << "[FallbackDriveToChargingStation] Published charger coordinate "
            << "timestamp: "
            << sent_timestamp_
            << std::endl;



        if (!getInput<double>("timeout", timeout_))
        {
            timeout_ = 5.0;
        }



        start_time_ =
            std::chrono::steady_clock::now();



        return BT::NodeStatus::RUNNING;
    }



    BT::NodeStatus onRunning() override
    {

        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                start_time_)
                .count();



        if (elapsed >= timeout_)
        {
            std::cout
                << "[FallbackDriveToChargingStation] Timeout ("
                << timeout_
                << "s) -> SUCCESS"
                << std::endl;


            return BT::NodeStatus::SUCCESS;
        }



        return BT::NodeStatus::RUNNING;
    }



    void onHalted() override
    {
        std::cout
            << "[FallbackDriveToChargingStation] HALTED"
            << std::endl;
    }



private:

    double timeout_;

    std::string sent_timestamp_;

    std::chrono::steady_clock::time_point start_time_;


    geometry_msgs::msg::PoseStamped sent_coord_;


    rclcpp::Node::SharedPtr node_;


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_coord_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
};

class RobotIsRobotAtChargingStation : public BT::StatefulActionNode
{
public:
    RobotIsRobotAtChargingStation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), timeout_(10.0)
    {
        node_ = rclcpp::Node::make_shared("btRobotIsRobotAtChargingStation");

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/drive_to_coord_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string data = msg->data;
                std::cout << "[RobotIsRobotAtChargingStation] Ontvangen bericht: "
                          << data << std::endl;

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

                // Alleen de eerste 10 cijfers van de timestamp vergelijken
                std::string expected_prefix = sent_timestamp_.substr(0, 10);
                std::string recv_prefix = recv_timestamp.substr(0, 10);

                if (recv_prefix == expected_prefix)
                {
                    if (status_code == "04")
                    {
                        received_success_ = true;
                    }
                    // else if (status_code == "05" || status_code == "07")
                    // {
                    //     received_failure_ = true;
                    //     std::cout << "[RobotIsRobotAtChargingStation] FAILURE ontvangen"
                    //               << std::endl;
                    // }
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("charging_sent_timestamp"),
            BT::InputPort<bool>("skip_drive2charging"),
            BT::OutputPort<bool>("drive_failed"),
            BT::InputPort<bool>("skip_robotdrivechargingstation"),

        };
    }

    BT::NodeStatus onStart() override
    {
        // Eerst controleren of rijden moet worden overgeslagen
        bool skip = false;
        getInput("skip_drive2charging", skip);

        if (skip)
        {
            std::cout << "[RobotIsRobotAtChargingStation] skip_drive2charging = TRUE -> SUCCESS"
                      << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        bool skip_robot_drive = false;
        if (getInput("skip_robotdrivechargingstation", skip_robot_drive))
        {
            if (skip_robot_drive)
            {
                std::cout << "[RobotIsRobotAtChargingStation] skip_robotdrivechargingstation = TRUE -> SUCCESS"
                        << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }


        received_success_ = false;
        received_failure_ = false;

        start_time_ = std::chrono::steady_clock::now();

        if (!getInput<double>("timeout", timeout_))
            timeout_ = 10.0;

        if (!getInput<std::string>("charging_sent_timestamp", sent_timestamp_))
        {
            std::cout << "[RobotIsRobotAtChargingStation] Geen timestamp ontvangen van blackboard!"
                      << std::endl;
        }
        else
        {
            std::cout << "[RobotIsRobotAtChargingStation] Verwachte timestamp = "
                      << sent_timestamp_ << std::endl;
        }

        std_msgs::msg::String msg;
        msg.data = "RobotIsRobotAtChargingStation";
        pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        auto elapsed = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start_time_)
                           .count();

        if (received_success_)
        {
            std::cout << "[RobotIsRobotAtChargingStation] Successtatus ontvangen -> SUCCESS"
                      << std::endl;

            setOutput("drive_failed", false);

            return BT::NodeStatus::SUCCESS;
        }

        if (received_failure_)
        {
            std::cout << "[RobotIsRobotAtChargingStation] Faalstatus ontvangen -> FAILURE"
                      << std::endl;

            setOutput("drive_failed", true);

            return BT::NodeStatus::FAILURE;
        }

        if (elapsed >= timeout_)
        {
            std::cout << "[RobotIsRobotAtChargingStation] Timeout ("
                      << timeout_ << "s) -> FAILURE" << std::endl;

            setOutput("drive_failed", true);

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[RobotIsRobotAtChargingStation] HALTED" << std::endl;
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





class DriveToChargingStation : public BT::StatefulActionNode
{
public:
    DriveToChargingStation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          success_received_(false),
          force_charge_received_(false),
          timeout_(5.0)
    {
        node_ = rclcpp::Node::make_shared("btDriveToChargingStation");

        rclcpp::QoS qos(1);
        qos.reliable();

        // ==========================
        // Subscriber auto recharge
        // ==========================
        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/auto_recharge_event", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::cout << "\n[CALLBACK] Nieuw bericht ontvangen: "
                          << msg->data << std::endl;

                if(msg->data.size() < 2)
                {
                    std::cout << "[CALLBACK] Bericht te kort -> negeren" << std::endl;
                    return;
                }

                int msg_id = msg->data[0] - '0';
                std::string event = msg->data.substr(1);

                std::cout << "[CALLBACK] Parsed msg_id: "
                          << msg_id
                          << " | event: "
                          << event << std::endl;

                int bt_id = 0;
                if(!getInput("chargingInteger", bt_id))
                {
                    std::cout << "[CALLBACK] chargingInteger niet gevonden." << std::endl;
                }

                if(msg_id != bt_id)
                {
                    std::cout << "[CALLBACK] msg_id != bt_id -> genegeerd" << std::endl;
                    return;
                }

                if(event == "DRIVING-TO-DOCK")
                {
                    std::cout << "[CALLBACK] DRIVING-TO-DOCK ontvangen." << std::endl;

                    setOutput("skip_statusDriveToChargingStation", false);
                    setOutput("skip_isrobotcharging", false);
                    success_received_ = true;
                }
                else if(event == "DRIVE-TO-DOCK-SUCCESS")
                {
                    std::cout << "[CALLBACK] DRIVE-TO-DOCK-SUCCESS ontvangen." << std::endl;

                    setOutput("skip_statusDriveToChargingStation", true);
                    setOutput("skip_isrobotcharging", false);


                    success_received_ = true;
                }
                else if(event == "ROBOT-CHARGING")
                {
                        std::cout << "[CALLBACK] ROBOT-CHARGING ontvangen." << std::endl;

                        setOutput("skip_isrobotcharging", true);
                        setOutput("skip_statusDriveToChargingStation", true);

                        success_received_ = true;
                }
            });

        // ==========================
        // Echo subscriber force charge
        // ==========================
        force_charge_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/force_charge", qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::cout << "[FORCE_CHARGE CALLBACK] Ontvangen: "
                          << msg->data << std::endl;

                if(msg->data == "0START")
                {
                    force_charge_received_ = true;
                    std::cout << "[FORCE_CHARGE CALLBACK] Echo OK." << std::endl;
                }
            });

        rclcpp::QoS docking_qos(1);
        docking_qos.reliable();
        docking_qos.transient_local();


        // ==========================
        // Publishers
        // ==========================
        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);

        force_charge_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/force_charge", 10);

        infrared_docking_pub_ =
    node_->create_publisher<std_msgs::msg::String>(
        "/infrared_docking_status",docking_qos);


    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<int>("chargingInteger"),
            BT::OutputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("connection_chargeStatus"),
            BT::InputPort<bool>("skip_drive2charging"),
            BT::OutputPort<bool>("skip_statusDriveToChargingStation"),
            BT::OutputPort<bool>("skip_isrobotcharging"),

        };
    }

    BT::NodeStatus onStart() override
    {
        publishDockingEnabled();

        success_received_ = false;
        force_charge_received_ = false;

        start_time_ = std::chrono::steady_clock::now();

        getInput("timeout", timeout_);

        setOutput("connection_chargeStatus", true);

        //---------------------------------
        // Skip check
        //---------------------------------
        bool skip = false;

        if(getInput("skip_drive2charging", skip))
        {
            if(skip)
            {
                std::cout << "[DriveToChargingStation] Robot is al aan het laden (skip)" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }

        //---------------------------------
        // Blackboard check
        //---------------------------------
        int bt_id = 0;

        if(!getInput("chargingInteger", bt_id))
        {
            std::cout << "[DriveToChargingStation] chargingInteger ontbreekt."
                      << std::endl;
        }

        setOutput("bat_admin_status", "STOP");

        //---------------------------------
        // Andere publishers
        //---------------------------------
        std_msgs::msg::String msg;
        msg.data = "DriveToChargingStation";
        pub_->publish(msg);

        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotGoCharge";
        pub_quiz_->publish(quiz_msg);

        //---------------------------------
        // Force charge sturen
        //---------------------------------
        std_msgs::msg::String force_msg;
        force_msg.data = "0START";

        std::cout << "[DriveToChargingStation] Verstuur force_charge #1"
                  << std::endl;
        force_charge_pub_->publish(force_msg);

        rclcpp::spin_some(node_);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::cout << "[DriveToChargingStation] Verstuur force_charge #2"
                  << std::endl;
        force_charge_pub_->publish(force_msg);

        rclcpp::spin_some(node_);

        //---------------------------------
        // Wachten op echo
        //---------------------------------
        auto wait_start = std::chrono::steady_clock::now();

        while(rclcpp::ok())
        {
            rclcpp::spin_some(node_);

            if(force_charge_received_)
            {
                std::cout << "[DriveToChargingStation] Echo ontvangen op /force_charge"
                          << std::endl;
                break;
            }

            auto elapsed =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - wait_start)
                    .count();

            if(elapsed > 2.0)
            {
                std::cout << "[DriveToChargingStation] Geen echo ontvangen."
                          << std::endl;

                setOutput("connection_chargeStatus", false);

                return BT::NodeStatus::FAILURE;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "[DriveToChargingStation] Wachten op DRIVING-TO-DOCK..."
                  << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if(success_received_)
        {
            std::cout << "[DriveToChargingStation] SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time_)
                .count();

        if(elapsed >= timeout_)
        {
            std::cout << "[DriveToChargingStation] TIMEOUT" << std::endl;
            
            setOutput("skip_statusDriveToChargingStation", false);
            setOutput("skip_isrobotcharging", false);

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[DriveToChargingStation] HALTED" << std::endl;
    }

    void publishDockingEnabled()
{
    std_msgs::msg::String docking_msg;
    docking_msg.data = "DOCKING_ENABLED";


    while(infrared_docking_pub_->get_subscription_count() == 0)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Waiting for subscribers on /infrared_docking_status..."
        );

        rclcpp::sleep_for(
            std::chrono::milliseconds(100));
    }


    infrared_docking_pub_->publish(docking_msg);


    std::cout 
        << "[DriveToChargingStation] Published DOCKING_ENABLED"
        << std::endl;
}



private:
    bool success_received_;
    bool force_charge_received_;

    double timeout_;

    std::chrono::steady_clock::time_point start_time_;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr force_charge_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr infrared_docking_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_charge_pub_;
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

        // Publisher om een klein stukje vooruit te rijden
        pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("gui_cmd_vel", 10);
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

        while (pub_coord_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Waiting for subscribers on /btDriveCoord...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        pub_coord_->publish(sent_coord_);

        // Timestamp opslaan
        sent_timestamp_ =
            std::to_string(sent_coord_.header.stamp.sec) + "." +
            std::to_string(sent_coord_.header.stamp.nanosec);

        setOutput("workarea_timestamp", sent_timestamp_);

        std::cout << "[DriveWorkArea] Published coordinate at timestamp: "
                  << sent_timestamp_ << std::endl;

        // ----------------------------------------------------------
        // Rijd een klein stukje vooruit (~0.5 meter)
        // ----------------------------------------------------------

        while (pub_cmd_vel_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Waiting for subscribers on gui_cmd_vel...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.20;   // rustige snelheid
        cmd.angular.z = 0.0;

        auto drive_start = std::chrono::steady_clock::now();

        while (std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - drive_start)
                   .count() < 1.0)
        {
            pub_cmd_vel_->publish(cmd);
            rclcpp::spin_some(node_);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop de robot
        geometry_msgs::msg::Twist stop_cmd;
        pub_cmd_vel_->publish(stop_cmd);

        // ----------------------------------------------------------

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
        // Zorg ervoor dat de robot zeker stopt
        geometry_msgs::msg::Twist stop_cmd;
        pub_cmd_vel_->publish(stop_cmd);

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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
};


class FallbackDriveToWorkArea : public BT::StatefulActionNode
{
public:
    FallbackDriveToWorkArea(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        // Interne ROS 2 node-naam aangepast
        node_ = rclcpp::Node::make_shared("btFallbackDriveToWorkArea");

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
        // Gepubliceerde BT node-naam aangepast naar FallbackDriveToWorkArea
        std_msgs::msg::String bt_msg;
        bt_msg.data = "FallbackDriveToWorkArea";
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

        while (pub_coord_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Waiting for subscribers on /btDriveCoord...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        pub_coord_->publish(sent_coord_);

        // Timestamp opslaan
        sent_timestamp_ = std::to_string(sent_coord_.header.stamp.sec) + "." +
                          std::to_string(sent_coord_.header.stamp.nanosec);

        setOutput("workarea_timestamp", sent_timestamp_);

        // Lognaam aangepast naar [FallbackDriveToWorkArea]
        std::cout << "[FallbackDriveToWorkArea] Published coordinate at timestamp: "
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
            // Lognaam aangepast naar [FallbackDriveToWorkArea]
            std::cout << "[FallbackDriveToWorkArea] Timeout ("
                      << timeout_ << "s) -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // Lognaam aangepast naar [FallbackDriveToWorkArea]
        std::cout << "[FallbackDriveToWorkArea] HALTED" << std::endl;
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
    RobotExplore(const std::string& name,
                 const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btRobotExplore");

        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);
    }


    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("robotLocation"),

            BT::OutputPort<std::string>("explore_timestamp"),

            // Nieuwe input
            BT::InputPort<bool>("show_verdwaald")
        };
    }


    BT::NodeStatus tick() override
    {

        // =====================================================
        // Check show_verdwaald blackboard
        // =====================================================

        bool show_verdwaald = false;

        getInput("show_verdwaald", show_verdwaald);


        if (show_verdwaald)
        {
            std::cout
                << "[RobotExplore] show_verdwaald = TRUE -> checking status file"
                << std::endl;


            const std::string status_file =
                "/home/wheeltec/wheeltec_ros2/src/april_tabloo/status.txt";


            std::ifstream file(status_file);


            if (file.is_open())
            {
                std::string status;

                std::getline(file, status);

                file.close();


                // spaties/newlines verwijderen
                status.erase(
                    std::remove_if(status.begin(),
                                   status.end(),
                                   [](unsigned char c)
                                   {
                                       return std::isspace(c);
                                   }),
                    status.end());


                std::cout
                    << "[RobotExplore] status.txt = "
                    << status
                    << std::endl;



                if (status == "NOK")
                {
                    std::cout
                        << "[RobotExplore] Status NOK -> FAILURE"
                        << std::endl;

                    return BT::NodeStatus::FAILURE;
                }


                if (status == "OK")
                {
                    std::cout
                        << "[RobotExplore] Status OK -> normale werking"
                        << std::endl;
                }
            }
            else
            {
                std::cout
                    << "[RobotExplore] Kan status file niet openen -> normale werking"
                    << std::endl;
            }
        }



        // =====================================================
        // Originele RobotExplore logica
        // =====================================================


        setOutput("robotLocation", "WORKING");


        // timestamp maken (Unix time)
        auto stamp = node_->get_clock()->now();


        int64_t sec =
            static_cast<int64_t>(stamp.seconds());


        int64_t nanosec =
            stamp.nanoseconds() % 1000000000;



        std::string explore_timestamp =
            std::to_string(sec)
            + "."
            + std::to_string(nanosec);



        setOutput("explore_timestamp",
                  explore_timestamp);



        std::string state = "RobotExplore";


        std_msgs::msg::String msg;

        msg.data = state;

        pub_quiz_->publish(msg);



        std_msgs::msg::String bt_msg;

        bt_msg.data = "RobotExplore";

        pub_bt_->publish(bt_msg);



        std::cout
            << "[RobotExplore] Exploring environment (sim)"
            << std::endl;


        std::cout
            << "[RobotExplore] Timestamp: "
            << explore_timestamp
            << std::endl;



        return BT::NodeStatus::SUCCESS;
    }


private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
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
            BT::InputPort<bool>("skip_drive2charging"),
            BT::InputPort<bool>("skip_statusDriveToChargingStation"), 
            BT::InputPort<bool>("skip_isrobotcharging")

        };
    }

    BT::NodeStatus onStart() override
    {


        bool skip_status = false;
        if (getInput("skip_statusDriveToChargingStation", skip_status))
        {
            if (skip_status)
            {
                std::cout << "[StatusDriveToChargingDock] skip want drive-to-dock-success is al gepasseerd" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }


        // Nieuwe check
        bool skip_robot_charging = false;
        if (getInput("skip_isrobotcharging", skip_robot_charging))
        {
            if (skip_robot_charging)
            {
                std::cout << "[StatusDriveToChargingDock] skip want ROBOT-CHARGING is reeds gepasseerd" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }


        status_ = "";
        getInput("timeout", timeout_);
        start_time_ = std::chrono::steady_clock::now();

        
        // check skip_drive2charging bij start
        bool skip = false;
        if (getInput("skip_drive2charging", skip))
        {
            if (skip)
            {
                std::cout << "[StatusDriveToChargingDock] skip want robot is reeds aan het laden" << std::endl;
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

                if (msg_id != bt_id)
                    return;

                event_ = event;
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);

        // NEW: RPI topic publisher
        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>("/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<int>("chargingInteger"),
            BT::InputPort<bool>("skip_drive2charging"),
            BT::InputPort<bool>("skip_isrobotcharging")
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

        // NEW: publish RPI screen
        std_msgs::msg::String rpi_msg;
        rpi_msg.data = "RobotDocking";
        rpi_pub_->publish(rpi_msg);

        bool skip = false;
        if (getInput("skip_drive2charging", skip))
        {
            if (skip)
            {
                std::cout << "[IsRobotCharging] skip robot reeds aan het laden" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                std::cout << "[IsRobotCharging] skip_drive2charging = FALSE -> " << std::endl;
            }
        }


        // Nieuwe check
        bool skip_robot_charging = false;
        if (getInput("skip_isrobotcharging", skip_robot_charging))
        {
            if (skip_robot_charging)
            {
                std::cout << "[IsRobotCharging] skip want robot charging is reeds gepasseerd als bericht" << std::endl;
                return BT::NodeStatus::SUCCESS;
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

    void onHalted() override
    {
        std::cout << "[IsRobotCharging] HALTED" << std::endl;
    }

private:
    std::string event_;
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

    // NEW
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;
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

    bool batteryOkInFile()
    {
        const std::string filePath = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt";
        std::ifstream file(filePath);

        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line == "BATTERY-OK")
            {
                return true;
            }
        }

        return false;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("robotLocation"),
            BT::InputPort<int>("chargingInteger"),
            BT::OutputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("skip_isrobotcharging"),
            BT::OutputPort<bool>("skip_robotdrivechargingstation")
        };
    }

    BT::NodeStatus onStart() override
    {
        setOutput("skip_isrobotcharging", false);
        setOutput("robotLocation", "CHARGING");
        setOutput("bat_admin_status", "START");
        setOutput("skip_robotdrivechargingstation", false);

        // Stuur bericht naar rpitopic
        std_msgs::msg::String msg;
        msg.data = "RobotCharging";
        pub_quiz_->publish(msg);

        std_msgs::msg::String bt_msg;
        bt_msg.data = "IsBatteryFull";
        pub_bt_->publish(bt_msg);

        // ===== NIEUW: schrijf PASS naar file =====
        const std::string file_path =
            "/home/wheeltec/wheeltec_ros2/src/robot_position_reset/robot_position_reset/manual_mode.txt";

        std::ofstream file(file_path, std::ios::out | std::ios::trunc);
        if (file.is_open())
        {
            file << "PASS";
            file.close();
            std::cout << "[IsBatteryFull] wrote PASS to file" << std::endl;
        }
        else
        {
            std::cout << "[IsBatteryFull] FAILED to open file" << std::endl;
        }
        // ========================================

        rclcpp::spin_some(node_);

        if (batteryOkInFile())
        {
            std::cout << "[IsBatteryFull] BATTERY-OK gevonden in bestand." << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (batteryOkInFile())
        {
            std::cout << "[IsBatteryFull] BATTERY-OK gevonden in bestand." << std::endl;
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

#include <fstream>
#include <fstream>
#include <random>
#include <fstream>
#include <random>
#include <cctype>
class CheckingNearbyVisitors : public BT::StatefulActionNode
{
public:

    CheckingNearbyVisitors(const std::string &name,
                           const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          received_drive_to_quiz_(false)
    {
        node_ = rclcpp::Node::make_shared("btCheckingNearbyVisitors");

        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        search_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/search_cmd_vel", 10);

        // -----------------------------
        // Target distance subscriber
        // -----------------------------
        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

        sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/target_distance",
            qos,
            [this](std_msgs::msg::Float32::SharedPtr msg)
            {
                latest_value_ = msg->data;
            });

        // -----------------------------
        // Quiz knop subscriber
        // -----------------------------
        rclcpp::QoS quiz_qos(1);
        quiz_qos.reliable();
        quiz_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        sub_quiz_ = node_->create_subscription<std_msgs::msg::String>(
            "/quiz",
            quiz_qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string expected =
                    "drive_to_quiz_location" + visitor_code_;

                if(msg->data == expected)
                {
                    std::cout
                    << "[CheckingNearbyVisitors] Received correct quiz code: "
                    << msg->data
                    << std::endl;

                    received_drive_to_quiz_ = true;

                    std::ofstream file(
                    "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");

                    if(file.is_open())
                    {
                        file << "GEDRUKT";
                        file.close();
                    }
                }
                else
                {
                    std::cout
                    << "[CheckingNearbyVisitors] Wrong quiz code received: "
                    << msg->data
                    << std::endl;
                }

            });
    }

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<double>("timer"),
            BT::InputPort<std::string>("visitor_code"),
            BT::OutputPort<bool>("robot_needs_rotation")
        };
    }

    BT::NodeStatus onStart() override
    {
        // -----------------------------
        // Visitor code ophalen via InputPort
        // (wordt nu al gegenereerd door StartDrivingToPeople)
        // -----------------------------
        visitor_code_.clear();

        if(getInput("visitor_code", visitor_code_))
        {
            std::cout
                << "[CheckingNearbyVisitors] Visitor code received from port: "
                << visitor_code_
                << std::endl;
        }
        else
        {
            std::cout
                << "[CheckingNearbyVisitors] WARNING: No visitor_code received"
                << std::endl;
        }

        // standaard: volgende nodes uitvoeren
        setOutput("robot_needs_rotation", true);

        //--------------------------------------------------------
        // Controle: knop mogelijk al ingedrukt vóór deze node
        // (scherm staat al aan sinds StartDrivingToPeople)
        //--------------------------------------------------------
        {
            std::ifstream file(
            "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");

            if(file.is_open())
            {
                std::string status;

                std::getline(file, status);

                file.close();

                if(status == "GEDRUKT")
                {
                    std::cout
                        << "[CheckingNearbyVisitors] quizknop.txt = GEDRUKT -> SUCCESS"
                        << std::endl;

                    setOutput("robot_needs_rotation", false);

                    return BT::NodeStatus::SUCCESS;
                }
            }
        }

        latest_value_ = 999.0;
        received_drive_to_quiz_ = false;

        start_time_ =
            std::chrono::steady_clock::now();

        rotation_start_time_ =
            std::chrono::steady_clock::now();

        std_msgs::msg::String msg;
        msg.data = "CheckingNearbyVisitors";
        pub_->publish(msg);

        std::cout
        << "[CheckingNearbyVisitors] Visitor code: "
        << visitor_code_
        << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        //--------------------------------------------------------
        // Quiz knop correct ingedrukt -> volgende nodes skippen
        //--------------------------------------------------------
        if(received_drive_to_quiz_)
        {
            stopRotation();

            setOutput("robot_needs_rotation", false);

            std::cout
            << "[CheckingNearbyVisitors] QUIZ BUTTON CORRECT -> SUCCESS"
            << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        double timer = 30.0;
        getInput("timer", timer);

        //--------------------------------------------------------
        // Roteren
        //--------------------------------------------------------
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.angular.z = 0.25;

        search_pub_->publish(cmd);

        //--------------------------------------------------------
        // Persoon gezien
        //--------------------------------------------------------
        if(latest_value_ != 999.0 &&
           latest_value_ > 0.0)
        {
            stopRotation();

            setOutput("robot_needs_rotation", true);

            std::cout
            << "[CheckingNearbyVisitors] PERSON DETECTED -> SUCCESS"
            << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        //--------------------------------------------------------
        // Kwartrotatie klaar
        //--------------------------------------------------------
        auto rotation_elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now()
            - rotation_start_time_)
            .count();

        double quarter_rotation_time = 38;

        if(rotation_elapsed >= quarter_rotation_time)
        {
            stopRotation();

            setOutput("robot_needs_rotation", true);

            std::cout
            << "[CheckingNearbyVisitors] QUARTER ROTATION COMPLETE"
            << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        //--------------------------------------------------------
        // Timeout
        //--------------------------------------------------------
        auto elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now()
            - start_time_)
            .count();

        if(elapsed >= timer)
        {
            stopRotation();

            setOutput("robot_needs_rotation", true);

            std::cout
            << "[CheckingNearbyVisitors] TIMEOUT"
            << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        stopRotation();

        std::cout
        << "[CheckingNearbyVisitors] HALTED"
        << std::endl;
    }

private:

    void stopRotation()
    {
        geometry_msgs::msg::Twist stop;

        stop.linear.x = 0.0;
        stop.angular.z = 0.0;

        search_pub_->publish(stop);
    }

private:

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr search_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_quiz_;

    float latest_value_ = 999.0;

    bool received_drive_to_quiz_;

    std::string visitor_code_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point rotation_start_time_;
};


#include <fstream>
#include <string>
// BT node die bepaalt of robot de visitors bereikt heeft via afstand + trigger event
class ArrivedAtVisitors : public BT::StatefulActionNode
{
public:

    ArrivedAtVisitors(const std::string &name,
                      const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(15.0),
          received_drive_to_quiz_(false),
          overlimit_count_(0),
          follow_value_(0.0)
    {

        node_ = rclcpp::Node::make_shared("btArrivedAtVisitors");


        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode",
            10);


        pub_quiz_screen_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/rpitopic",
                10);



        sub_follow_ =
            node_->create_subscription<std_msgs::msg::Float32>(
                "/target_distance",
                10,
                [this](std_msgs::msg::Float32::SharedPtr msg)
                {
                    follow_value_ = msg->data;
                });



        rclcpp::QoS qos(1);
        qos.reliable();
        qos.durability(
            RMW_QOS_POLICY_DURABILITY_VOLATILE);



        sub_quiz_ =
            node_->create_subscription<std_msgs::msg::String>(
                "/quiz",
                qos,
                [this](std_msgs::msg::String::SharedPtr msg)
                {

                    std::string expected_message =
                        "drive_to_quiz_location" + visitor_code_;


                    if(msg->data == expected_message)
                    {

                        std::cout
                            << "[ArrivedAtVisitors] Correct quiz code received: "
                            << msg->data
                            << std::endl;


                        received_drive_to_quiz_ = true;


                        std::ofstream file(
                            "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");


                        if(file.is_open())
                        {
                            file << "GEDRUKT";
                            file.close();
                        }

                    }
                    else
                    {

                        std::cout
                            << "[ArrivedAtVisitors] Wrong quiz message ignored: "
                            << msg->data
                            << std::endl;

                    }

                });
    }





    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("visitor_code"),
            BT::OutputPort<bool>("robot_needs_rotation")
        };
    }






    BT::NodeStatus onStart() override
    {

        visitor_code_.clear();


        if(getInput("visitor_code", visitor_code_))
        {

            std::cout
                << "[ArrivedAtVisitors] Visitor code received: "
                << visitor_code_
                << std::endl;

        }



        //--------------------------------------------------------
        // standaard: volgende nodes mogen draaien
        //--------------------------------------------------------

        setOutput("robot_needs_rotation", true);






        //--------------------------------------------------------
        // Eerst controleren of knop al gedrukt was
        //--------------------------------------------------------

        {

            std::ifstream file(
                "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");


            if(file.is_open())
            {

                std::string status;

                std::getline(file,status);

                file.close();



                if(status == "GEDRUKT")
                {

                    std::cout
                        << "[ArrivedAtVisitors] quizknop.txt = GEDRUKT -> SUCCESS"
                        << std::endl;


                    // volgende nodes skippen
                    setOutput("robot_needs_rotation", false);


                    return BT::NodeStatus::SUCCESS;
                }

            }
        }






        overlimit_count_ = 0;

        received_drive_to_quiz_ = false;

        follow_value_ = 0.0;





        if(!getInput<double>("timeout", timeout_))
        {
            timeout_ = 15.0;
        }





        start_time_ =
            std::chrono::steady_clock::now();






        std_msgs::msg::String msg_bt_;

        msg_bt_.data = "ArrivedAtVisitors";

        pub_bt_->publish(msg_bt_);






        std_msgs::msg::String screen_msg;


        screen_msg.data =
            "RobotArrivedAtVisitors" + visitor_code_;


        pub_quiz_screen_->publish(screen_msg);





        std::cout
            << "[ArrivedAtVisitors] START timeout="
            << timeout_
            << " visitor_code="
            << visitor_code_
            << std::endl;






        return BT::NodeStatus::RUNNING;
    }









    BT::NodeStatus onRunning() override
    {

        rclcpp::spin_some(node_);




        std_msgs::msg::String msg_bt_;

        msg_bt_.data = "ArrivedAtVisitors";

        pub_bt_->publish(msg_bt_);






        //--------------------------------------------------------
        // Correct quiz bericht ontvangen
        // -> volgende nodes skippen
        //--------------------------------------------------------

        if(received_drive_to_quiz_)
        {

            setOutput("robot_needs_rotation", false);


            std::cout
                << "[ArrivedAtVisitors] Correct quiz code -> SUCCESS"
                << std::endl;



            return BT::NodeStatus::SUCCESS;
        }







        //--------------------------------------------------------
        // Filter afstand
        //--------------------------------------------------------

        if(follow_value_ > 3.0)
        {

            overlimit_count_++;

        }
        else
        {

            overlimit_count_ = 0;

        }






        if(overlimit_count_ >= 5)
        {

            setOutput("robot_needs_rotation", true);


            std::cout
                << "[ArrivedAtVisitors] 5 measurements > 3.0 -> FAILURE"
                << std::endl;



            return BT::NodeStatus::FAILURE;

        }









        //--------------------------------------------------------
        // Timeout
        //--------------------------------------------------------

        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                start_time_)
            .count();





        if(elapsed >= timeout_)
        {

            setOutput("robot_needs_rotation", true);


            std::cout
                << "[ArrivedAtVisitors] Timeout "
                << elapsed
                << "s -> FAILURE"
                << std::endl;



            return BT::NodeStatus::FAILURE;

        }





        return BT::NodeStatus::RUNNING;
    }








    void onHalted() override
    {

        std::cout
            << "[ArrivedAtVisitors] HALTED"
            << std::endl;

    }







private:


    double timeout_;

    bool received_drive_to_quiz_;

    int overlimit_count_;

    float follow_value_;


    std::string visitor_code_;


    std::chrono::steady_clock::time_point start_time_;


    rclcpp::Node::SharedPtr node_;


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_screen_;


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

        pub_rpi_ = node_->create_publisher<std_msgs::msg::String>(
        "/rpitopic", 10);



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


        // Toon FollowRobotScreen op de RPi
        std_msgs::msg::String screen_msg;
        screen_msg.data = "FollowRobotScreen";
        pub_rpi_->publish(screen_msg);



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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_rpi_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_coord_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tracking_enable_;
};

// BT node die een doelpositie naar quiz locatie stuurt via PoseStamped (Fallback variant)
class FallbackDriveQuizLocation : public BT::StatefulActionNode
{
public:
    FallbackDriveQuizLocation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        // Interne ROS 2 node-naam aangepast
        node_ = rclcpp::Node::make_shared("btFallbackDriveQuizLocation");

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
        // Gepubliceerde BT node-naam aangepast naar FallbackDriveQuizLocation
        std_msgs::msg::String bt_msg;
        bt_msg.data = "FallbackDriveQuizLocation";
        pub_bt_->publish(bt_msg);

        // stop follow-me/person tracking
        std_msgs::msg::Bool tracking_msg;
        tracking_msg.data = false;
        pub_tracking_enable_->publish(tracking_msg);

        std::cout << "[FallbackDriveQuizLocation] Tracking DISABLED" << std::endl;

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

        std::cout << "[FallbackDriveQuizLocation] Published coordinate at timestamp: "
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
            std::cout << "[FallbackDriveQuizLocation] Timeout (" << timeout_ << "s) -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[FallbackDriveQuizLocation] HALTED" << std::endl;
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
                    // else if (status_code == "05" ||  status_code == "07"){
                    //     received_failure_ = true;
                    //     std::cout << "FAILURE ONTVANGEN";
                    // }
                   
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

class FallbackIsRobotAtChargingStation : public BT::StatefulActionNode
{
public:

    FallbackIsRobotAtChargingStation(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          timeout_(10.0),
          received_success_(false),
          received_failure_(false)
    {

        node_ = rclcpp::Node::make_shared(
            "btFallbackIsRobotAtChargingStation");


        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/drive_to_coord_status",
            10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {

                std::string data = msg->data;


                std::cout
                    << "[FallbackIsRobotAtChargingStation] Ontvangen bericht: "
                    << data
                    << std::endl;



                std::vector<std::string> parts;

                std::stringstream ss(data);

                std::string segment;


                while (std::getline(ss, segment, '-'))
                {
                    parts.push_back(segment);
                }



                if (parts.size() < 2)
                {
                    return;
                }



                std::string status_code = parts[0];

                std::string recv_timestamp = parts[1];



                // Alleen eerste 10 cijfers vergelijken
                std::string expected_prefix =
                    sent_timestamp_.substr(0, 10);


                std::string recv_prefix =
                    recv_timestamp.substr(0, 10);



                if (recv_prefix == expected_prefix)
                {

                    if (status_code == "04")
                    {
                        received_success_ = true;


                        std::cout
                            << "[FallbackIsRobotAtChargingStation] "
                            << "Correcte laadstation aankomst ontvangen"
                            << std::endl;
                    }

                }

            });



        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode",
            10);
    }



    static BT::PortsList providedPorts()
    {
        return {

            BT::InputPort<double>("timeout"),

            BT::InputPort<std::string>(
                "charging_sent_timestamp"),

            BT::OutputPort<bool>(
                "drive_failed")
        };
    }




    BT::NodeStatus onStart() override
    {

        received_success_ = false;

        received_failure_ = false;



        start_time_ =
            std::chrono::steady_clock::now();



        if (!getInput<double>("timeout", timeout_))
        {
            timeout_ = 10.0;
        }




        if (!getInput<std::string>(
                "charging_sent_timestamp",
                sent_timestamp_))
        {

            std::cout
                << "[FallbackIsRobotAtChargingStation] "
                << "Geen timestamp ontvangen van blackboard!"
                << std::endl;

        }
        else
        {

            std::cout
                << "[FallbackIsRobotAtChargingStation] "
                << "Verwachte timestamp = "
                << sent_timestamp_
                << std::endl;

        }




        std_msgs::msg::String msg;

        msg.data =
            "FallbackIsRobotAtChargingStation";

        pub_->publish(msg);



        return BT::NodeStatus::RUNNING;
    }





    BT::NodeStatus onRunning() override
    {

        rclcpp::spin_some(node_);



        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                start_time_)
                .count();





        if (received_success_)
        {

            std::cout
                << "[FallbackIsRobotAtChargingStation] "
                << "Successtatus ontvangen -> SUCCESS"
                << std::endl;



            setOutput(
                "drive_failed",
                false);



            return BT::NodeStatus::SUCCESS;

        }





        if (received_failure_)
        {

            std::cout
                << "[FallbackIsRobotAtChargingStation] "
                << "Faalstatus ontvangen -> FAILURE"
                << std::endl;



            setOutput(
                "drive_failed",
                true);



            return BT::NodeStatus::FAILURE;

        }





        if (elapsed >= timeout_)
        {

            std::cout
                << "[FallbackIsRobotAtChargingStation] Timeout ("
                << timeout_
                << "s) -> FAILURE"
                << std::endl;



            setOutput(
                "drive_failed",
                true);



            return BT::NodeStatus::FAILURE;

        }





        return BT::NodeStatus::RUNNING;
    }





    void onHalted() override
    {

        std::cout
            << "[FallbackIsRobotAtChargingStation] HALTED"
            << std::endl;

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

class FallbackIsRobotAtQuiz : public BT::StatefulActionNode
{
public:
    FallbackIsRobotAtQuiz(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), timeout_(10.0)
    {
        // Interne ROS 2 node-naam aangepast
        node_ = rclcpp::Node::make_shared("btFallbackIsRobotAtQuiz");

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/drive_to_coord_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string data = msg->data;
                std::cout << "[FallbackIsRobotAtQuiz] Ontvangen bericht: " << data << std::endl;

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
                    {
                        received_success_ = true;
                    }
                    // else if (status_code == "05" || status_code == "07")
                    // {
                    //     received_failure_ = true;
                    //     std::cout << "[FallbackIsRobotAtQuiz] FAILURE ONTVANGEN" << std::endl;
                    // }
                }
            });

        pub_ = node_->create_publisher<std_msgs::msg::String>("/BehaviorTreeNode", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("timeout"),
            BT::InputPort<std::string>("sent_timestamp"), // Timestamp uit blackboard
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

        if (!getInput<std::string>("sent_timestamp", sent_timestamp_))
            std::cout << "[FallbackIsRobotAtQuiz] Geen timestamp ontvangen van blackboard!" << std::endl;
        else
            std::cout << "[FallbackIsRobotAtQuiz] Verwachte timestamp = " << sent_timestamp_ << std::endl;

        // Gepubliceerde BT node-naam aangepast naar FallbackIsRobotAtQuiz
        std_msgs::msg::String msg;
        msg.data = "FallbackIsRobotAtQuiz";
        pub_->publish(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

        if (received_success_)
        {
            std::cout << "[FallbackIsRobotAtQuiz] Successtatus ontvangen -> SUCCESS" << std::endl;
            setOutput("drive_failed", false);
            return BT::NodeStatus::SUCCESS;
        }

        if (received_failure_)
        {
            std::cout << "[FallbackIsRobotAtQuiz] Faalstatus ontvangen -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (elapsed >= timeout_)
        {
            std::cout << "[FallbackIsRobotAtQuiz] Timeout (" << timeout_ << "s) -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[FallbackIsRobotAtQuiz] HALTED" << std::endl;
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


class RobotFailedDriveToChargingStation : public BT::StatefulActionNode
{
public:
    RobotFailedDriveToChargingStation(const std::string &name,
                                      const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("btRobotFailedDriveToChargingStation");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("skip_robotdrivechargingstation")
        };
    }

    BT::NodeStatus onStart() override
    {
        // Blackboard variabele zetten
        setOutput("skip_robotdrivechargingstation", true);

        // Publiceer naam van de BT-node
        std_msgs::msg::String bt_msg;
        bt_msg.data = "RobotFailedDriveToChargingStation";
        pub_bt_->publish(bt_msg);

        // Publiceer scherm voor de RPi
        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotFailedDriveToCharging";
        pub_quiz_->publish(quiz_msg);

        // ===== NIEUW: schrijf SKIP naar file =====
        const std::string file_path =
            "/home/wheeltec/wheeltec_ros2/src/robot_position_reset/robot_position_reset/manual_mode.txt";

        std::ofstream file(file_path, std::ios::out | std::ios::trunc);
        if (file.is_open())
        {
            file << "SKIP";
            file.close();
            std::cout << "[RobotFailedDriveToChargingStation] wrote SKIP to file" << std::endl;
        }
        else
        {
            std::cout << "[RobotFailedDriveToChargingStation] FAILED to open file" << std::endl;
        }
        // ========================================

        std::cout << "[RobotFailedDriveToChargingStation] "
                  << "skip_robotdrivechargingstation = TRUE"
                  << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[RobotFailedDriveToChargingStation] HALTED" << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
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

        force_charge_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/force_charge", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("buttonStop"),

            BT::OutputPort<bool>("skip_drive2charging"),
            BT::OutputPort<bool>("skip_robotdrivechargingstation"),

            BT::OutputPort<std::string>("robotLocationBAT"),
            BT::OutputPort<std::string>("bat_admin_status"),
            BT::OutputPort<bool>("robot_startup"),
            BT::OutputPort<bool>("skip_drivetoworkarea")  
        };
    }

    BT::NodeStatus tick() override
    {
        // 🔹 ALTIJD FALSE SCHRIJVEN NAAR BLACKBOARD
        setOutput("robot_startup", false);
        setOutput("skip_drivetoworkarea", false);

        
        // 1. Publiceer node naam
        std_msgs::msg::String msg;
        msg.data = "CheckButtonState";
        pub_->publish(msg);

        
        // Haal de data op (let op de accolades voor de vector)
        json statusData = retrieveRobotStatus({"robotActive"});

        bool robot_active = false; // Fallback waarde
        if (statusData.contains("robotActive") && !statusData["robotActive"].is_null()) {
            robot_active = statusData["robotActive"].get<bool>();
        }

        std::cout << "Werk/Slaap status van de robot opgehaald uit DB: " 
          << (robot_active ? "true" : "false") << std::endl;

        // 4. Gedrag (vervanging van START/STOP button)
        if (robot_active)
        {
            std::cout << "[CheckButtonState] START toestand (robot actief)" << std::endl;
            setOutput("buttonStop", false);
            setOutput("robotLocationBAT", std::string("WORKING"));
            // HIER ALLES RESETTEN VAN ANDERE DINGEN ZODAT ROBOT NIET VALSSPEELT

            setOutput("skip_drive2charging", false);
            setOutput("skip_robotdrivechargingstation", false);
            setOutput("bat_admin_status", std::string("STOP"));

                    std_msgs::msg::String cmd_msg;
        cmd_msg.data = "0STOP";

        for (int i = 0; i < 3; ++i)
        {
            force_charge_pub_->publish(cmd_msg);
        }

        std::cout << "[CheckButtonState] Published STOP to /force_charge (knop inschakelen)" << std::endl;
        


            // OOK DAT ROBOT LADEN STOPT MOET HIER GEBEUREN
        }
        else
        {
            std::cout << "[CheckButtonState] STOP toestand (robot NIET actief)" << std::endl;
            setOutput("buttonStop", true);
            setOutput("robotLocationBAT", std::string("FORCE-CHARGING"));
            
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_charge_pub_;
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

        // if (!getInput("chargingInteger", counter))
        // {
        //     throw BT::RuntimeError("chargingInteger ontbreekt");
        // }

        if (counter == 9){
            counter = 0;
        }
        else{
            counter += 1;
        }
        
        std::cout << "[StopRobotCharging] regel voor setoutput chargingintegernextcycle" << std::endl;

        //setOutput("chargingInteger_nextCycle", 0);

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
        cmd_msg.data = "0STOP";

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

                // IR docking status publisher
        rclcpp::QoS docking_qos(1);
        docking_qos.reliable();
        docking_qos.transient_local();

        infrared_docking_pub_ =
            node_->create_publisher<std_msgs::msg::String>(
                "/infrared_docking_status",
                docking_qos);
    }

    void publishDockingDisabled()
    {
        std_msgs::msg::String docking_msg;
        docking_msg.data = "DOCKING_DISABLED";


        while(infrared_docking_pub_->get_subscription_count() == 0)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Waiting for subscribers on /infrared_docking_status..."
            );

            rclcpp::sleep_for(
                std::chrono::milliseconds(100));
        }


        infrared_docking_pub_->publish(docking_msg);


        std::cout
            << "[BatteryCharged] Published DOCKING_DISABLED"
            << std::endl;
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
        publishDockingDisabled();

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
            std::cout << "[BatteryCharged] Timeout reached RETURN FROM CHARGING gezet op ROBOTLOCATIONBat -> SUCCESS"
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr infrared_docking_pub_;
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
                    // else if (status_code == "05" || status_code == "07")
                    // {
                    //     received_failure_ = true;
                    //     std::cout << "[IsRobotAtWorkArea] FAILURE ONTVANGEN" << std::endl;
                    // }
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

class FallbackIsRobotAtWorkArea : public BT::StatefulActionNode
{
public:
    FallbackIsRobotAtWorkArea(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), timeout_(10.0)
    {
        // Interne ROS 2 node-naam aangepast
        node_ = rclcpp::Node::make_shared("btFallbackIsRobotAtWorkArea");

        sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/drive_to_coord_status", 10,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string data = msg->data;
                std::cout << "[FallbackIsRobotAtWorkArea] Ontvangen bericht: " << data << std::endl;

                if (data.size() >= 7 && data.substr(0, 7) == "12-0000")
                {
                    received_failure_ = true;
                    std::cout << "[FallbackIsRobotAtWorkArea] FAILURE door 12-0000 prefix" << std::endl;
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
                    // else if (status_code == "05" || status_code == "07")
                    // {
                    //     received_failure_ = true;
                    //     std::cout << "[FallbackIsRobotAtWorkArea] FAILURE ONTVANGEN" << std::endl;
                    // }
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
            std::cout << "[FallbackIsRobotAtWorkArea] Geen timestamp ontvangen van blackboard!" << std::endl;
        else
            std::cout << "[FallbackIsRobotAtWorkArea] Verwachte timestamp = " << sent_timestamp_ << std::endl;

        // Gepubliceerde BT node-naam aangepast naar FallbackIsRobotAtWorkArea
        std_msgs::msg::String msg;
        msg.data = "FallbackIsRobotAtWorkArea";
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
            std::cout << "[FallbackIsRobotAtWorkArea] Successtatus ontvangen -> SUCCESS" << std::endl;
            setOutput("drive_failed", false);
            return BT::NodeStatus::SUCCESS;
        }

        if (received_failure_)
        {
            std::cout << "[FallbackIsRobotAtWorkArea] Faalstatus ontvangen -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (elapsed >= timeout_)
        {
            std::cout << "[FallbackIsRobotAtWorkArea] Timeout (" 
                      << timeout_ << "s) -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[FallbackIsRobotAtWorkArea] HALTED" << std::endl;
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


#include <fstream>
#include <string>

#include <fstream>
class RobotRotationFollowMe : public BT::StatefulActionNode
{
public:

    RobotRotationFollowMe(const std::string &name,
                          const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          received_drive_to_quiz_(false)
    {
        node_ = rclcpp::Node::make_shared("bt_robot_rotation_follow_me");


        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode",
            10);



        rclcpp::QoS qos(1);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);



        rclcpp::QoS quiz_qos(1);
        quiz_qos.reliable();
        quiz_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);





        sub_distance_ =
            node_->create_subscription<std_msgs::msg::Float32>(
                "/target_distance",
                qos,
                [this](std_msgs::msg::Float32::SharedPtr msg)
                {
                    latest_distance_ = msg->data;
                    new_measurement_ = true;
                });

        // -------------------------------------------------
        // Quiz knop subscriber
        // -------------------------------------------------

        sub_quiz_ =
            node_->create_subscription<std_msgs::msg::String>(
                "/quiz",
                quiz_qos,
                [this](std_msgs::msg::String::SharedPtr msg)
                {

                    std::string expected_message =
                        "drive_to_quiz_location" + visitor_code_;

                    if(msg->data == expected_message)
                    {
                        std::cout
                            << "[RobotRotationFollowMe] Correct quiz code received: "
                            << msg->data
                            << std::endl;
                        received_drive_to_quiz_ = true;

                        std::ofstream file(
                            "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");


                        if(file.is_open())
                        {
                            file << "GEDRUKT";
                            file.close();
                        }
                    }
                    else
                    {
                        std::cout
                            << "[RobotRotationFollowMe] Wrong quiz message ignored: "
                            << msg->data
                            << std::endl;
                    }

                });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("distance_max"),
            BT::InputPort<double>("timer"),
            BT::InputPort<int>("zero_limit"),
            BT::InputPort<std::string>("visitor_code"),
            
            BT::OutputPort<bool>("robot_needs_rotation")
        };
    }

    BT::NodeStatus onStart() override
    {

        //--------------------------------------------------------
        // Visitor code ophalen via InputPort
        //--------------------------------------------------------

        visitor_code_.clear();


        if(getInput("visitor_code", visitor_code_))
        {
            std::cout
                << "[RobotRotationFollowMe] Visitor code received from port: "
                << visitor_code_
                << std::endl;
        }
        else
        {
            std::cout
                << "[RobotRotationFollowMe] WARNING: No visitor_code received"
                << std::endl;
        }

        //--------------------------------------------------------
        // Controle vorige knop
        //--------------------------------------------------------

        {
            std::ifstream file(
                "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");

            if(file.is_open())
            {
                std::string status;

                std::getline(file,status);

                file.close();



                if(status == "GEDRUKT")
                {
                    std::cout
                        << "[RobotRotationFollowMe] quizknop.txt = GEDRUKT -> SUCCESS"
                        << std::endl;

                    setOutput("robot_needs_rotation", false);


                    return BT::NodeStatus::SUCCESS;
                }
            }
        }

        latest_distance_ = 0.0;

        new_measurement_ = false;

        consecutive_zero_count_ = 0;

        received_drive_to_quiz_ = false;



        setOutput("robot_needs_rotation", true);

        getInput("distance_max", distance_max_);

        getInput("timer", wait_duration_);

        getInput("zero_limit", zero_limit_);





        start_time_ =
            std::chrono::steady_clock::now();






        std_msgs::msg::String bt_msg;

        bt_msg.data = "RobotRotationFollowMe";

        pub_bt_->publish(bt_msg);






        std::cout
            << "[RobotRotationFollowMe] START"
            << " distance_max="
            << distance_max_
            << " timer="
            << wait_duration_
            << " zero_limit="
            << zero_limit_
            << " visitor_code="
            << visitor_code_
            << std::endl;





        return BT::NodeStatus::RUNNING;
    }









    BT::NodeStatus onRunning() override
    {

        rclcpp::spin_some(node_);





        //--------------------------------------------------------
        // Correct quiz bericht ontvangen
        //--------------------------------------------------------

        if(received_drive_to_quiz_)
        {

            std::cout
                << "[RobotRotationFollowMe] Correct quiz button -> SUCCESS"
                << std::endl;


            setOutput("robot_needs_rotation", false);

            return BT::NodeStatus::SUCCESS;
        }







        //--------------------------------------------------------
        // Nieuwe afstandsmeting
        //--------------------------------------------------------

        if(new_measurement_)
        {

            new_measurement_ = false;



            if(latest_distance_ == 0.0)
            {

                consecutive_zero_count_++;



                std::cout
                    << "[RobotRotationFollowMe] Zero measurement "
                    << consecutive_zero_count_
                    << "/"
                    << zero_limit_
                    << std::endl;





                if(consecutive_zero_count_ >= zero_limit_)
                {

                    std::cout
                        << "[RobotRotationFollowMe] Zero limit reached -> FAILURE"
                        << std::endl;

                    setOutput("robot_needs_rotation", true);

                    return BT::NodeStatus::FAILURE;
                }

            }

            else
            {

                consecutive_zero_count_ = 0;



                if(latest_distance_ < distance_max_)
                {

                    std::cout
                        << "[RobotRotationFollowMe] Person found at "
                        << latest_distance_
                        << " m -> SUCCESS"
                        << std::endl;


                    setOutput("robot_needs_rotation", true);

                    return BT::NodeStatus::SUCCESS;
                }

            }

        }

        //--------------------------------------------------------
        // Timeout
        //--------------------------------------------------------

        auto elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now()
                -
                start_time_)
            .count();

        if(elapsed >= wait_duration_)
        {

            std::cout
                << "[RobotRotationFollowMe] Timer expired -> FAILURE"
                << std::endl;


            setOutput("robot_needs_rotation", true);

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout
            << "[RobotRotationFollowMe] HALTED"
            << std::endl;
    }

private:

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_distance_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_quiz_;

    float latest_distance_ = 0.0;
    bool new_measurement_ = false;
    int consecutive_zero_count_ = 0;

    double distance_max_ = 1.7;


    double wait_duration_ = 50.0;


    int zero_limit_ = 8;




    bool received_drive_to_quiz_ = false;



    // Code ontvangen via InputPort
    std::string visitor_code_;




    std::chrono::steady_clock::time_point start_time_;
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
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
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
        return {
            BT::InputPort<double>("timeout"),
            BT::OutputPort<bool>("skip_drivetoworkarea") 
        };
    }

    BT::NodeStatus onStart() override
    {

        setOutput("skip_drivetoworkarea", true);
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

        // Publisher voor BT-status
        pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("drive_failed"),
            BT::OutputPort<bool>("show_verdwaald")
        };
    }

    BT::NodeStatus onStart() override
    {
        // =====================================================
        // Toon verdwaald melding
        // =====================================================
        setOutput("show_verdwaald", true);

        // =====================================================
        // Publiceer de node-status
        // =====================================================
        std_msgs::msg::String msg;
        msg.data = "MainBTStopDrive";
        pub_->publish(msg);

        // =====================================================
        // RPI topic publish
        // =====================================================
        std_msgs::msg::String rpi_msg;
        rpi_msg.data = "RobotError";
        rpi_pub_->publish(rpi_msg);

        std::cout
            << "[MainBTStopDrive] Node gestart, show_verdwaald = TRUE"
            << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        bool drive_failed = false;

        if (!getInput<bool>("drive_failed", drive_failed))
        {
            std::cout
                << "[MainBTStopDrive] Geen drive_failed waarde gevonden, ga van FALSE uit"
                << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        if (drive_failed)
        {
            std::cout
                << "[MainBTStopDrive] drive_failed = TRUE, blijf RUNNING"
                << std::endl;

            return BT::NodeStatus::RUNNING;
        }
        else
        {
            std::cout
                << "[MainBTStopDrive] drive_failed = FALSE, returning SUCCESS"
                << std::endl;

            return BT::NodeStatus::SUCCESS;
        }
    }

    void onHalted() override
    {
        std::cout
            << "[MainBTStopDrive] HALTED"
            << std::endl;
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
        rpi_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("drive_failed"),
            BT::OutputPort<bool>("show_verdwaald")
        };
    }

    BT::NodeStatus onStart() override
    {
        // =====================================================
        // Zet blackboard flags
        // =====================================================
        setOutput("drive_failed", true);
        setOutput("show_verdwaald", false);

        // =====================================================
        // Publish naar RPI topic
        // =====================================================
        std_msgs::msg::String msg;
        msg.data = "RobotStarting";
        rpi_pub_->publish(msg);

        // =====================================================
        // Start tijd registreren
        // =====================================================
        start_time_ = std::chrono::steady_clock::now();

        std::cout
            << "[MainBTSetErrorFlag] started -> publishing RobotStarting"
            << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        auto seconds =
            std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

        if (seconds >= 5)
        {
            std::cout
                << "[MainBTSetErrorFlag] 5 seconds done -> SUCCESS"
                << std::endl;

            return BT::NodeStatus::SUCCESS;
        }

        std::cout
            << "[MainBTSetErrorFlag] running... ("
            << seconds
            << "s)"
            << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout
            << "[MainBTSetErrorFlag] HALTED"
            << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpi_pub_;

    std::chrono::steady_clock::time_point start_time_;
};



#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <thread>
#include <chrono>
class StartDrivingToPeople : public BT::StatefulActionNode
{
public:
    StartDrivingToPeople(const std::string &name,
                         const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config),
          received_drive_to_quiz_(false)
    {
        node_ = rclcpp::Node::make_shared("btStartDrivingToPeople");

        pub_bt_ = node_->create_publisher<std_msgs::msg::String>(
            "/BehaviorTreeNode", 10);

        pub_quiz_ = node_->create_publisher<std_msgs::msg::String>(
            "/rpitopic", 10);

        pub_tracking_enable_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/tracking_enable", 10);

        pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/gui_cmd_vel", 10);

        // -----------------------------
        // Quiz knop subscriber
        // -----------------------------
        rclcpp::QoS quiz_qos(1);
        quiz_qos.reliable();
        quiz_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        sub_quiz_ = node_->create_subscription<std_msgs::msg::String>(
            "/quiz",
            quiz_qos,
            [this](std_msgs::msg::String::SharedPtr msg)
            {
                std::string expected =
                    "drive_to_quiz_location" + visitor_code_;

                if(msg->data == expected)
                {
                    std::cout
                    << "[StartDrivingToPeople] Received correct quiz code: "
                    << msg->data
                    << std::endl;

                    received_drive_to_quiz_ = true;

                    std::ofstream file(
                    "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");

                    if(file.is_open())
                    {
                        file << "GEDRUKT";
                        file.close();
                    }
                }
                else
                {
                    std::cout
                    << "[StartDrivingToPeople] Wrong quiz code received: "
                    << msg->data
                    << std::endl;
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("explore_timestamp"),
            BT::InputPort<int>("delta_seconds"),
            BT::InputPort<bool>("robot_needs_rotation"),
            BT::OutputPort<bool>("show_verdwaald"),
            BT::OutputPort<std::string>("visitor_code")
        };
    }

    BT::NodeStatus onStart() override
    {
        // =====================================================
        // Reset verdwaald melding
        // =====================================================
        setOutput("show_verdwaald", false);

        // =====================================================
        // Nieuwe visitor code + knopscherm activeren
        // =====================================================
        visitor_code_ = generateRandomCode();
        setOutput("visitor_code", visitor_code_);
        received_drive_to_quiz_ = false;

        {
            std::ofstream file(
            "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/quizknop.txt");

            if(file.is_open())
            {
                file << "LOS";
                file.close();
            }
        }

        std_msgs::msg::String quiz_msg;
        quiz_msg.data = "RobotExplore";
        pub_quiz_->publish(quiz_msg);

        std_msgs::msg::String screen_msg;
        screen_msg.data = "RobotArrivedAtVisitors" + visitor_code_;
        pub_quiz_->publish(screen_msg);

        std::cout
        << "[StartDrivingToPeople] Screen trigger sent: "
        << screen_msg.data
        << std::endl;

        std::cout
        << "[StartDrivingToPeople] Visitor code: "
        << visitor_code_
        << std::endl;

        // =====================================================
        // Indien nodig eerst een kwartslag draaien
        // =====================================================
        bool robot_needs_rotation = false;

        if (getInput("robot_needs_rotation", robot_needs_rotation) &&
            robot_needs_rotation)
        {
            std::cout << "[StartDrivingToPeople] Performing quarter rotation..."
                      << std::endl;

            geometry_msgs::msg::Twist twist;
            twist.angular.z = 0.3;

            rclcpp::Rate rate(20);

            // ongeveer 90 graden draaien
            // 0.3 rad/s gedurende ±5.2 s ≈ 1.57 rad
            for (int i = 0; i < 104; i++)
            {
                pub_cmd_vel_->publish(twist);
                rate.sleep();
            }

            // Stoppen
            twist.angular.z = 0.0;
            pub_cmd_vel_->publish(twist);

            std::cout << "[StartDrivingToPeople] Quarter rotation finished"
                      << std::endl;
        }

        // =====================================================
        // Publish naar BehaviorTreeNode
        // =====================================================
        std_msgs::msg::String bt_msg;
        bt_msg.data = "StartDrivingToPeople";
        pub_bt_->publish(bt_msg);

        // =====================================================
        // Tracking inschakelen
        // =====================================================
        std_msgs::msg::Bool tracking_msg;
        tracking_msg.data = true;
        pub_tracking_enable_->publish(tracking_msg);

        std::cout << "[StartDrivingToPeople] Tracking ENABLED" << std::endl;
        std::cout << "[StartDrivingToPeople] Started driving to people"
                  << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        // =====================================================
        // TIMESTAMP CHECK
        // =====================================================
        std::string ts_str;
        int delta_seconds = 0;

        if (getInput("explore_timestamp", ts_str) &&
            getInput("delta_seconds", delta_seconds))
        {
            try
            {
                size_t dot = ts_str.find('.');

                if (dot != std::string::npos)
                {
                    long sec = std::stol(ts_str.substr(0, dot));

                    auto now = node_->get_clock()->now();
                    long now_sec = now.seconds();

                    long diff = std::abs(now_sec - sec);

                    if (diff > delta_seconds)
                    {
                        std::cout
                            << "[StartDrivingToPeople] TIMEOUT detected (diff="
                            << diff
                            << "s > delta="
                            << delta_seconds
                            << ") -> restart_tree=true"
                            << std::endl;

                        config().blackboard->set("restart_tree", true);

                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
            catch (const std::exception &e)
            {
                std::cout
                    << "[StartDrivingToPeople] Timestamp parse error: "
                    << e.what()
                    << std::endl;
            }
        }

        // =====================================================
        // Klaar?
        // =====================================================

        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        std_msgs::msg::Bool tracking_msg;
        tracking_msg.data = false;
        pub_tracking_enable_->publish(tracking_msg);

        geometry_msgs::msg::Twist stop;
        pub_cmd_vel_->publish(stop);

        std::cout << "[StartDrivingToPeople] Tracking DISABLED" << std::endl;
        std::cout << "[StartDrivingToPeople] HALTED" << std::endl;
    }

private:

    std::string generateRandomCode()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());

        std::uniform_int_distribution<int> letter_dist(0,25);
        std::uniform_int_distribution<int> number_dist(0,9);

        char letter =
            'A' + letter_dist(gen);

        int number =
            number_dist(gen);

        return std::string(1, letter)
               + std::to_string(number);
    }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_bt_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_quiz_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tracking_enable_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_quiz_;

    bool received_drive_to_quiz_;
    std::string visitor_code_;
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

                    timer_started_ = true;
                    start_time_ = std::chrono::steady_clock::now();
                }
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("robot_startup"),
            BT::InputPort<std::string>("bat_admin_status"),
            BT::InputPort<int>("chargingInteger"),
            BT::OutputPort<int>("chargingInteger_nextCycle")
        };
    }


    BT::NodeStatus onStart() override
    {
        // 🔹 BLACKBOARD CHECK
        bool robot_startup;
        if (!getInput("robot_startup", robot_startup))
        {
            std::cout << "[CheckAdminCondition] robot_startup NIET gevonden -> SUCCESS" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        admin_closed_ = false;
        manual_drive_ = false;
        timer_started_ = false;

        std_msgs::msg::String msg;
        msg.data = "CheckAdminCondition";
        pub_->publish(msg);

        std::string bat_status;
        if (getInput("bat_admin_status", bat_status))
        {
            if (bat_status == "STOP")
            {
                std::cout << "[CheckAdminCondition] bat_admin_status = STOP" << std::endl;


            }
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (admin_closed_)
        {
            if (timer_started_)
            {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();

                if (elapsed >= 2)
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
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("robot_startup")
        };
    }

    BT::NodeStatus onStart() override
    {
        // CHECK BLACKBOARD VARIABLE robot_startup
        bool robot_startup;
        if (!getInput("robot_startup", robot_startup))
        {
            std::cout << "[CheckAdminPanel] robot_startup NIET gevonden -> FAILURE" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        admin_panel_open_ = false;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // Check admin panel status via database
        json status = retrieveRobotStatus({"adminPanelIsOpen"});

        if (status.contains("adminPanelIsOpen"))
        {
            admin_panel_open_ = status["adminPanelIsOpen"].get<bool>();
        }
        else
        {
            std::cout << "[CheckAdminPanel] adminPanelIsOpen niet gevonden in database" << std::endl;
        }

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
    bool admin_panel_open_;
};

/* RT: we willen de robot ook kunnen opstarten aan de april TAG.

class CheckAprilTagLocalization : public BT::StatefulActionNode
{
public:
    CheckAprilTagLocalization(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_apriltag_check");

        sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
            {
                last_pose_ = *msg;
                initial_pose_received_ = true;
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("robotLocationBAT"),
            BT::OutputPort<std::string>("robotLocation")
        };
    }

    BT::NodeStatus onStart() override
    {
        initial_pose_received_ = false;
        start_time_ = node_->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        // 1. Tag gezien → SUCCESS
        if (initial_pose_received_)
        {
            setOutput("robotLocationBAT", "WORKING");
            setOutput("robotLocation", "WORKING");
            initial_pose_received_ = false;
            return BT::NodeStatus::SUCCESS;
        }

        // 2. Timeout → FAILURE
        auto elapsed = node_->now() - start_time_;
        if (elapsed.seconds() > 5.0)
        {
            return BT::NodeStatus::SUCCESS;
        }

        // 3. Nog bezig → RUNNING
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
    bool initial_pose_received_ = false;
    rclcpp::Time start_time_;
};

*/

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
    
    factory.registerNodeType<RobotRotationFollowMe>("RobotRotationFollowMe");


    factory.registerNodeType<CheckNetworkError>("CheckNetworkError");
    factory.registerNodeType<CheckAdminPanel>("CheckAdminPanel");

    factory.registerNodeType<FallbackDriveQuizLocation>("FallbackDriveQuizLocation");
    factory.registerNodeType<FallbackIsRobotAtQuiz>("FallbackIsRobotAtQuiz");

    factory.registerNodeType<FallbackDriveToWorkArea>("FallbackDriveToWorkArea");
    factory.registerNodeType<FallbackIsRobotAtWorkArea>("FallbackIsRobotAtWorkArea");

    factory.registerNodeType<CheckAdminCondition>("CheckAdminCondition");
    factory.registerNodeType<CheckButtonState>("CheckButtonState");

    factory.registerNodeType<ConnectionLost >("ConnectionLost");

    factory.registerNodeType<StopRobotCharging>("StopRobotCharging");
    factory.registerNodeType<MainBTStopDrive>("MainBTStopDrive");
    factory.registerNodeType<MainBTSetErrorFlag>("MainBTSetErrorFlag");

    factory.registerNodeType<RobotFailedDriveToChargingStation>("RobotFailedDriveToChargingStation");


    factory.registerNodeType<ForceSuccess>("MainFallbackForceSuccess");
    factory.registerNodeType<ForceSuccess>("BatteryForceSuccess");


    factory.registerNodeType<RobotDriveToChargingStation>("RobotDriveToChargingStation");
    factory.registerNodeType<RobotIsRobotAtChargingStation>("RobotIsRobotAtChargingStation");

    factory.registerNodeType<FallbackIsRobotAtChargingStation>("FallbackIsRobotAtChargingStation");

    factory.registerNodeType<FallbackDriveToChargingStation>("FallbackDriveToChargingStation");


    factory.registerNodeType<LoopSequence>("LoopSequence");
    //factory.registerNodeType<CheckAprilTagLocalization>("CheckAprilTagLocalization");

  


    // laad boom uit XML
    auto tree = factory.createTreeFromFile("src/mecabot_bt/trees/behavior_tree.xml");
    

    tree.rootBlackboard()->set("restart_tree", false);

    std::cout << "--- Starting BT in continuous mode ---" << std::endl;
    rclcpp::Rate loop_rate(1.0); // definieer hoeveel ticks/sec naar rootnode gaan

    while (rclcpp::ok())
        {
            BT::NodeStatus status = tree.tickRoot();

            bool restart_tree = false;

            tree.rootBlackboard()->get("restart_tree", restart_tree);

            if (restart_tree)
            {
                std::cout << "=== RESTART TREE REQUESTED ===" << std::endl;

                // oude boom stoppen
                tree.rootNode()->halt();


                // flag resetten in nieuwe boom
                tree.rootBlackboard()->set("restart_tree", false);

                continue;
            }

            if (status == BT::NodeStatus::SUCCESS)
            {
                std::cout << "--- Tree ticked to SUCCESS ---" << std::endl;
                tree.rootNode()->halt();
            }
            else if (status == BT::NodeStatus::FAILURE)
            {
                std::cout << "--- Tree ticked to FAILURE ---" << std::endl;
                tree.rootNode()->halt();
            }

            loop_rate.sleep();
        }
    rclcpp::shutdown();
    return 0;
}




