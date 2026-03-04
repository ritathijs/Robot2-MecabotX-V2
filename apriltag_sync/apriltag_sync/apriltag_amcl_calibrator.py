#!/usr/bin/env python3
import os
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

# ROS 2 Berichten
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs # Vereist voor de .transform() functie

class AprilTagAmclCalibrator(Node):

    def __init__(self):
        super().__init__('apriltag_amcl_calibrator')

        # 1. Setup Publisher voor AMCL
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 2. Setup TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Laad de namen en locaties uit de config file
        self.tag_locations = self.load_tag_config()
        
        # 4. Instellingen voor stabiliteit
        self.cooldown_seconds = 5.0
        self.last_calibrated = self.get_clock().now()

        # 5. Timer (draait op 2Hz om de CPU niet te overbelasten)
        self.timer = self.create_timer(0.5, self.check_for_tags)

        self.get_logger().info("AprilTag AMCL Calibrator succesvol opgestart met eigen namen.")

    def load_tag_config(self):
        """Leest de tag_locations.yaml in om namen zoals 'laadstation' te vinden."""
        try:
            pkg_share = get_package_share_directory('apriltag_sync')
            config_path = os.path.join(pkg_share, 'config', 'tag_locations.yaml')
            
            with open(config_path, 'r') as f:
                content = yaml.safe_load(f)
                return content.get('tag_locations', {})
        except Exception as e:
            self.get_logger().error(f"Kon tag_locations.yaml niet laden: {str(e)}")
            return {}

    def check_for_tags(self):
        """Loop die controleert of er bekende tags in beeld zijn."""
        # Check cooldown
        elapsed = (self.get_clock().now() - self.last_calibrated).nanoseconds / 1e9
        if elapsed < self.cooldown_seconds:
            return

        for tag_id, info in self.tag_locations.items():
            tag_name = info.get('name')
            if not tag_name:
                continue

            # De namen die we nu gebruiken:
            detected_frame = tag_name          # bijv. 'laadstation' (van camera)
            static_frame = f"static_{tag_name}" # bijv. 'static_laadstation' (op de kaart)

            # Probeer te kalibreren als beide frames bestaan in TF
            if self.try_calibration(detected_frame, static_frame):
                self.last_calibrated = self.get_clock().now()
                break

    def try_calibration(self, detected_tag, static_tag):
        """Berekent de robot pose in 'map' op basis van een tag waarneming."""
        try:
            now = Time() # Gebruik de meest recente beschikbare transformatie

            # Controleer of de transformaties beschikbaar zijn
            if not self.tf_buffer.can_transform(detected_tag, 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=0.1)):
                return False

            # Stap 1: Waar is de robot ten opzichte van de tag die de camera nu ziet?
            # We maken een pose op (0,0,0) in het robot-frame
            pose_in_base = PoseStamped()
            pose_in_base.header.frame_id = 'base_footprint'
            pose_in_base.header.stamp = now.to_msg()
            pose_in_base.pose.orientation.w = 1.0

            # Transformeer de robot-positie naar het frame van de gedetecteerde tag
            pose_in_detected_tag = self.tf_buffer.transform(pose_in_base, detected_tag)
            
            # Stap 2: 'Plak' deze relatieve positie op het statische frame op de kaart
            # We veranderen het frame_id simpelweg naar de kaart-referentie
            pose_in_detected_tag.header.frame_id = static_tag

            # Stap 3: Bereken nu waar dit punt zich bevindt ten opzichte van 'map'
            robot_in_map = self.tf_buffer.transform(pose_in_detected_tag, 'map')

            # Stap 4: Maak de pose 2D (flatten) voor AMCL
            # AMCL accepteert geen 3D kantelingen
            q = robot_in_map.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # Stap 5: Publiceer naar /initialpose
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Positie (Z moet altijd 0 zijn voor AMCL)
            msg.pose.pose.position.x = robot_in_map.pose.position.x
            msg.pose.pose.position.y = robot_in_map.pose.position.y
            msg.pose.pose.position.z = 0.0

            # Oriëntatie (Alleen Yaw)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            # Covariantie (Onzekerheid instellen: hoe lager, hoe meer AMCL vertrouwt op de tag)
            msg.pose.covariance[0] = 0.1  # X onzekerheid
            msg.pose.covariance[7] = 0.1  # Y onzekerheid
            msg.pose.covariance[35] = 0.2 # Yaw onzekerheid

            self.pose_pub.publish(msg)
            self.get_logger().info(f"Kalibratie voltooid via tag: '{detected_tag}'")
            return True

        except TransformException as e:
            # Dit gebeurt vaak kortstondig tijdens het opstarten, dus we loggen het als debug
            return False
        except Exception as e:
            self.get_logger().error(f"Onverwachte fout bij kalibratie: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagAmclCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
