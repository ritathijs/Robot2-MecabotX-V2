#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

class StatusReader(Node):
    def __init__(self):
        super().__init__('status_reader')
        
        # Laatste BehaviorTreeNode (DriveQuizLocation, FallbackDriveQuizLocation, DriveWorkArea, ...)
        self.last_node = None
        
        # Interne docking- en gebruikersstatussen
        self.at_station = False
        self.is_charging = False
        self.robot_active = False  # Gebruikersstatus: Wil de gebruiker dat de robot werkt (True/False)
        
        # Lokalisatie status
        self.localization_valid = False
        self.received_amcl = False
        
        # Pad naar batterijstatus-file
        self.BATSTATUS_FILE = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt"

        # -----------------------------
        # SUBSCRIPTIONS
        # -----------------------------
        # AMCL covariance → lokalisatie geldigheid
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        # AprilTag reset done
        self.reset_done_sub = self.create_subscription(
            String,
            '/reset_done',
            self.reset_done_callback,
            10
        )
        
        # BT stuurt laatste node (Gevoed door robot_controller.cpp)
        self.bt_node_sub = self.create_subscription(
            String,
            '/BehaviorTreeNode',
            self.bt_node_callback,
            10
        )
        
        # Nav2 status van intacte drive_to_coord.py
        self.drive_status_sub = self.create_subscription(
            String,
            '/drive_to_coord_status',
            self.drive_status_callback,
            10
        )
        
        # Auto-recharger events
        self.recharge_event_sub = self.create_subscription(
            String,
            '/auto_recharge_event',
            self.recharge_event_callback,
            10
        )
        
        # Globale werkstatus (aan/uit van de gebruiker)
        self.robot_active_sub = self.create_subscription(
            Bool,
            '/RobotActive',
            self.robot_active_callback,
            10
        )

        # -----------------------------
        # PUBLISHERS NAAR BT
        # -----------------------------
        self.localization_pub = self.create_publisher(Bool, '/IsLocalizationValid', 10)
        self.quiz_pub = self.create_publisher(Bool, '/IsRobotAtQuiz', 10)
        self.work_pub = self.create_publisher(Bool, '/IsRobotAtWorkArea', 10)
        self.charge_pub = self.create_publisher(Bool, '/IsRobotAtChargingStation', 10)
        
        self.pub_is_charging = self.create_publisher(Bool, '/IsRobotCharging', 10)
        self.pub_robot_active = self.create_publisher(Bool, '/IsRobotActive', 10)
        
        self.battery_low_pub = self.create_publisher(Bool, '/IsBatteryLow', 10)
        self.battery_ok_pub = self.create_publisher(Bool, '/IsBatteryOK', 10)
        
        # Check elke 5 seconden de txt-file voor de accu
        self.battery_timer = self.create_timer(5.0, self.check_battery_status)
        
        self.get_logger().info("StatusReader gestart (Volledig afgestemd op intacte drive_to_coord).")

    # ============================================================
    # CALLBACKS
    # ============================================================
    def bt_node_callback(self, msg):
        self.last_node = msg.data.strip()
        self.get_logger().info(f"[BT] Node ontvangen: {self.last_node}")

    def drive_status_callback(self, msg):
        raw = msg.data
        try:
            status_code = int(raw[0:2])
        except:
            self.get_logger().error(f"[Nav2] Kon status niet parsen uit: {raw}")
            return
            
        # GECORRIGEERD: Omdat drive_to_coord.py intact blijft, luisteren we naar code 4 (GoalStatus.STATUS_SUCCEEDED)
        succeeded = (status_code == 4)
        
        self.get_logger().info(f"[Nav2] Status ontvangen: {status_code} (SUCCEEDED={succeeded})")
        
        # GECORRIGEERD: Nette rclpy-instantiatie om runtime crashes te voorkomen
        result_bool = Bool()
        result_bool.data = succeeded
        
        # GECORRIGEERD: Herkent nu de primaire EN de fallback-node uit de intacte drive_to_coord dictionary
        if self.last_node in ("DriveQuizLocation", "FallbackDriveQuizLocation"):
            self.quiz_pub.publish(result_bool)
        elif self.last_node == "DriveWorkArea":
            self.work_pub.publish(result_bool)
        elif self.last_node == "RobotDriveToChargingStation":
            self.charge_pub.publish(result_bool)

    # ============================================================
    # BATTERY STATUS
    # ============================================================
    def check_battery_status(self):
        if not os.path.exists(self.BATSTATUS_FILE):
            self.get_logger().warn("[Battery] batstatus.txt niet gevonden")
            return
        try:
            with open(self.BATSTATUS_FILE, "r") as f:
                status = f.read().strip()
        except Exception as e:
            self.get_logger().error(f"[Battery] fout bij lezen batstatus.txt: {e}")
            return
            
        low = (status == "BATTERY-LOW")
        ok = (status == "BATTERY-OK")
        
        # GECORRIGEERD: Veilig publiceren conform rclpy standaard
        msg_low = Bool()
        msg_low.data = low
        self.battery_low_pub.publish(msg_low)
        
        msg_ok = Bool()
        msg_ok.data = ok
        self.battery_ok_pub.publish(msg_ok)
        
        self.get_logger().info(f"[Battery] Status: {status}")

    # ============================================================
    # AUTO-RECHARGER EVENTS
    # ============================================================
    def recharge_event_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"[Docking] Event ontvangen: {data}")
        
        msg_true = Bool()
        msg_true.data = True
        
        if "DRIVE-TO-DOCK-SUCCESS" in data:
            self.at_station = True
            self.localization_valid = True
            self.publish_docking_booleans()
            self.localization_pub.publish(msg_true)
            
        if "ROBOT-CHARGING" in data:
            self.is_charging = True
            self.at_station = True
            self.robot_active = False # Tijdens het fysiek laden is hij niet autonoom actief
            self.localization_valid = True
            self.publish_docking_booleans()
            self.localization_pub.publish(msg_true)
            
        if "DOCK-FAILED" in data:
            self.at_station = False
            self.is_charging = False
            self.publish_docking_booleans()

    # ============================================================
    # ROBOT ACTIVE STATE (Globale werkmodus van gebruiker)
    # ============================================================
    def robot_active_callback(self, msg):
        self.robot_active = msg.data
        self.publish_docking_booleans()

    # ============================================================
    # PUBLISH DOCKING BOOLEANS
    # ============================================================
    def publish_docking_booleans(self):
        msg_charging = Bool()
        msg_charging.data = self.is_charging
        self.pub_is_charging.publish(msg_charging)
        
        msg_active = Bool()
        msg_active.data = self.robot_active
        self.pub_robot_active.publish(msg_active)
        
        msg_station = Bool()
        msg_station.data = self.at_station
        self.charge_pub.publish(msg_station)

    # ============================================================
    # APRILTAG RESET DONE
    # ============================================================
    def reset_done_callback(self, msg):
        if msg.data.lower() == "done":
            self.localization_valid = True
            msg_true = Bool()
            msg_true.data = True
            self.localization_pub.publish(msg_true)
            self.get_logger().info("[Localization] AprilTag reset geslaagd → valid = True")

    # ============================================================
    # AMCL CALLBACK
    # ============================================================
    def amcl_callback(self, msg):
        self.received_amcl = True
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        cov_yaw = msg.pose.covariance[35]
        
        amcl_good = (cov_x < 0.1 and cov_y < 0.1 and cov_yaw < 0.1)
        if amcl_good or self.at_station or self.is_charging:
            self.localization_valid = True
        else:
            self.localization_valid = False
            
        msg_loc = Bool()
        msg_loc.data = self.localization_valid
        self.localization_pub.publish(msg_loc)
        
        self.get_logger().info(
            f"[Localization] AMCL: cov=({cov_x:.3f}, {cov_y:.3f}, {cov_yaw:.3f}) → Valid={self.localization_valid}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StatusReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
