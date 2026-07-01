import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import math
import time  # nodig voor sleep


class PublishBackPose(Node):
    def __init__(self):
        super().__init__('publish_back_pose')

        
        self.manual_mode_file = "/home/wheeltec/wheeltec_ros2/src/robot_position_reset/robot_position_reset/manual_mode.txt"


        # QoS voor XSTOP met reliable (1 message buffer)
        qos_reliable = QoSProfile(depth=1)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE

        # publisher voor positie aan te passen
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # Publisher voor XSTOP met reliable QoS richting autochargecode
        self.xstop_pub = self.create_publisher(
            String,
            '/charge_XSTOP',
            qos_reliable
        )

        # Subscriber naar resetPositie naar autochargecode
        self.reset_sub = self.create_subscription(
            String,
            '/resetPositionChargeStation',
            self.reset_callback,
            1
        )

        # Subscriber voor MANUAL_DRIVE_CONTROL quiz_bt_node()
        self.manual_drive_sub = self.create_subscription(
            String,
            '/ManualDriveControleLocation',
            self.manual_drive_callback,
            1
        )

        self.reset_delay_timer = None


        # JSON bestand pad waar de locatie van het laadstation (+ 1.2m) instaat
        json_file = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json"
        with open(json_file, "r") as f:
            data = json.load(f)

        # positie uit json extraheren
        self.p_x = data["p_x"]
        self.p_y = data["p_y"]
        self.orien_z = data["orien_z"]
        self.orien_w = data["orien_w"]

        # Bereken yaw (rotatie in 2d vlak)
        self.yaw = 2 * math.atan2(self.orien_z, self.orien_w)

        # Bereken achterliggende positie 0.6m achter de coordinaat die in de json staat
        # de locatie in json is 1.2 m voor het station
        # bij een XSTOP rijdt de robot 0.6 m naar voor, waardoor hij dan tussen het laadstation en de coordinaat staat
        # er moet dus 0.6m van de coordinaat in json worden afgetrokken

        self.new_x = self.p_x - 0.6 * math.cos(self.yaw)
        self.new_y = self.p_y - 0.6 * math.sin(self.yaw)

        # timer voor 2 seconden te wachten, dan startup_callback uitvoeren
        self.start_timer = self.create_timer(2.0, self.startup_callback)
        self.start_timer.cancelled = False  # redundant

    def check_manual_mode(self):
        try:
            with open(self.manual_mode_file, "r") as f:
                return f.read().strip().upper()
        except Exception as e:
            self.get_logger().warn(f"Could not read manual mode file: {e}")
            return "PASS"  # fallback veilig gedrag
        


    def startup_callback(self):
        # Annuleer timer zodat dit maar 1 keer gebeurt
        self.start_timer.cancel()

        # Stuur XSTOPS richting autocharge om laden te stoppen
        self.send_xstopS()

    def send_xstopS(self):

        # wacht tot er minstens 1 subscriber is (enige code die luistert is autocharge)
        while self.xstop_pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for subscribers on /charge_XSTOP...")
            time.sleep(0.1)
        
        msg = String()
        msg.data = "XSTOPS"
        self.xstop_pub.publish(msg)
        self.get_logger().info("Sent XSTOPS to /force_charge (reliable)")


    def send_xstop(self):

        # wacht tot er minstens 1 subscriber is (enige code die luistert is autocharge)
        while self.xstop_pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for subscribers on /charge_XSTOP...")
            time.sleep(0.1)
        
        msg = String()
        msg.data = "XSTOP"
        self.xstop_pub.publish(msg)
        self.get_logger().info("Sent XSTOP to /force_charge (reliable)")

    def publish_pose(self):

        # maak een poseWithCovariantceStamped bericht aan volgens het formaat dat altijd gebruikt wordt
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # positie en orientatie invullen
        pose_msg.pose.pose.position.x = self.new_x
        pose_msg.pose.pose.position.y = self.new_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = self.orien_z
        pose_msg.pose.pose.orientation.w = self.orien_w

        # Covariance matrix initaliseren
        pose_msg.pose.covariance = [0.0]*36

        # onzekerheid is laag (we weten vrij exact waar robot staat)
        pose_msg.pose.covariance[0] = 0.01
        pose_msg.pose.covariance[7] = 0.01
        pose_msg.pose.covariance[35] = 0.01

        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Pose published: x={self.new_x:.3f}, y={self.new_y:.3f}')

    def publish_pose_multiple(self):
        # Publish 10 keer met kleine delay de locatie van chargestation
        for i in range(10):
            self.publish_pose()
            self.get_logger().info(f'Publish {i+1}/10 to /initialpose')
            time.sleep(0.3)  # kleine delay zodat AMCL het zeker ontvangt


    def reset_callback(self, msg):
        # callback voor reset berichten van autochargecode
        # deze wordt verstuurd als we XSTOP stuurden en de robot effectief aan het laden was
        # dit wil zeggen dat de locatie veranderd moet worden

        if msg.data == "RESET":
            self.get_logger().info("RESET received, waiting 3 seconds before republishing pose")

            # start delayed timer
            if self.reset_delay_timer is not None:
                self.reset_delay_timer.cancel()

            # wacht 4 seconden voor positie aan te passen (geef robot tijd om van laadstation af te bewegen)
            self.reset_delay_timer = self.create_timer(4.0, self.delayed_reset_publish)



    def delayed_reset_publish(self):
        self.get_logger().info("3 seconds passed after RESET, publishing pose")

        # timer meteen stoppen (belangrijk anders blijft hij herhalen)
        if self.reset_delay_timer is not None:
            self.reset_delay_timer.cancel()
            self.reset_delay_timer = None

        self.publish_pose_multiple()
        

    def manual_drive_callback(self, msg):
        if msg.data == "MANUAL_DRIVE_CONTROL":
            mode = self.check_manual_mode()

            self.get_logger().info(f"Manual drive received, mode = {mode}")

            if mode == "SKIP":
                self.get_logger().info("SKIP mode active → no XSTOP sent")
                return

            if mode == "PASS":
                self.get_logger().info("PASS mode → sending XSTOP")
                self.send_xstop()

def main(args=None):
    rclpy.init(args=args)
    node = PublishBackPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
