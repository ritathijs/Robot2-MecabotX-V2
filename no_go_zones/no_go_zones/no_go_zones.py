#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import cv2
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Bool
import struct

# --- INSTELLINGEN VOOR PIXEL RANGES ---
# Pas deze aan als je map net iets andere kleuren heeft

# 1. DE WEG (Wit)
# Alles met een waarde HOGER dan dit is 'weg' (vrij).
# Wit is normaal 255, maar door compressie vaak 250-254.
THRESHOLD_WEG_MIN = 225 

# 2. DE EXTENDED ZONE (Lichtgrijs)
# Dit moet een bereik zijn waar jouw lichtgrijze kleur (189) in valt.
# We pakken een ruime marge (bijv. 175 tot 205).
RANGE_EXTENDED_MIN = 175
RANGE_EXTENDED_MAX = 205

class DynamicObstaclePublisher(Node):

    def __init__(self, yaml_file):
        super().__init__('dynamic_obstacle_publisher')

        # --- 1. Parameters declareren ---
        self.declare_parameter('allow_extended_zone', True)
        
        # Lees de initiële status
        self.allow_extended_zone = self.get_parameter('allow_extended_zone').value
        self.get_logger().info(f"Initiële status allow_extended_zone: {self.allow_extended_zone}")

        # --- 2. YAML en PGM inlezen ---
        try:
            with open(yaml_file, 'r') as f:
                map_yaml = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Kon YAML niet laden: {yaml_file}. Fout: {e}")
            rclpy.shutdown()
            return

        yaml_dir = os.path.dirname(yaml_file)
        pgm_file_relative = map_yaml['image']
        pgm_file_absolute = os.path.join(yaml_dir, pgm_file_relative)

        self.resolution = map_yaml['resolution']
        self.origin_x, self.origin_y, _ = map_yaml['origin']

        self.get_logger().info(f"Kaart laden: {pgm_file_absolute}")
        self.get_logger().info(f"Resolutie: {self.resolution}, Origin: ({self.origin_x}, {self.origin_y})")

        # Lees afbeelding in (Grayscale)
        self.img = cv2.imread(pgm_file_absolute, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            self.get_logger().error(f"Kon afbeelding {pgm_file_absolute} niet laden!")
            rclpy.shutdown()
            return
            
        self.img_height = self.img.shape[0]

        # --- 3. Publisher ---
        # Transient Local is belangrijk zodat nieuwe subscribers (zoals Nav2 of RViz) 
        # de kaart direct krijgen, ook al zijn ze later ingestapt.
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.pub = self.create_publisher(PointCloud2, '/no_go_zones/floor_obstacles', qos_profile)

        # --- 4. Callbacks ---
        # Voor parameter updates (bijv. via CLI)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Voor live updates via topic (bijv. vanuit GUI)
        self.subscription = self.create_subscription(
            Bool,
            '/allow_extended_zone',
            self.topic_callback,
            10
        )

        # --- 5. Publiceer de eerste keer ---
        self.publish_obstacles()

    # --- CALLBACKS ---

    def topic_callback(self, msg):
        """Wordt aangeroepen bij een bericht op /control/allow_extended_zone"""
        new_state = msg.data
        
        if new_state != self.allow_extended_zone:
            self.get_logger().info(f"TOPIC: Schakelen naar {new_state}. Direct publiceren...")
            
            # 1. Update intern
            self.allow_extended_zone = new_state
            
            # 2. Publiceer nieuwe kaart direct
            self.publish_obstacles()
            
            # 3. Sync parameter op achtergrond
            try:
                self.set_parameters([Parameter('allow_extended_zone', Parameter.Type.BOOL, new_state)])
            except Exception:
                pass # Sync foutje is niet kritiek voor de werking

    def parameter_callback(self, params):
        """Wordt aangeroepen bij wijziging via 'ros2 param set' of RQT"""
        should_republish = False
        for param in params:
            if param.name == 'allow_extended_zone':
                if self.allow_extended_zone != param.value:
                    self.allow_extended_zone = param.value
                    self.get_logger().info(f"PARAM: Zone gewijzigd naar {self.allow_extended_zone}")
                    should_republish = True
        
        if should_republish:
            self.publish_obstacles()
            
        return SetParametersResult(successful=True)

    # --- CORE LOGICA ---

    def publish_obstacles(self):
        """Genereert de pointcloud op basis van pixelwaarden."""
        
        # DEBUG: Print de waarden die in je kaart zitten. 
        # Als je nog steeds ruis ziet, kijk dan of hier vreemde waarden tussen staan.
        unique_vals = np.unique(self.img)
        self.get_logger().info(f"DEBUG MAP PIXELS: {unique_vals}")

        # Startsituatie: ALLES is obstakel (255)
        mask = np.full(self.img.shape, 255, dtype=np.uint8)

        # 1. Maak de 'gewone' weg vrij (Wit)
        # Alles wat lichter is dan de threshold wordt 0 (geen obstakel)
        mask[self.img >= THRESHOLD_WEG_MIN] = 0

        # 2. Maak de 'extended' weg vrij (Lichtgrijs) - Alleen als True
        if self.allow_extended_zone:
            # Alles BINNEN het grijze bereik wordt 0
            mask[(self.img >= RANGE_EXTENDED_MIN) & (self.img <= RANGE_EXTENDED_MAX)] = 0
            self.get_logger().info("Zone status: OPEN (Grijs is vrij)")
        else:
            self.get_logger().info("Zone status: DICHT (Grijs is obstakel)")

        # Nu halen we alle coördinaten op die nog op 255 (obstakel) staan
        y_coords, x_coords = np.where(mask == 255)

        points = []
        for px, py in zip(x_coords, y_coords):
            # Formule: world = origin + pixel * resolutie
            world_x = self.origin_x + px * self.resolution
            # Y-as van afbeelding staat vaak "op zijn kop" t.o.v. wereldcoördinaten
            world_y = self.origin_y + (self.img_height - py - 1) * self.resolution
            points.append([world_x, world_y, 0.0])

        # Maak en publiceer bericht
        if not points:
            pc2_msg = self.create_pointcloud2([])
        else:
            pc2_msg = self.create_pointcloud2(points)

        self.pub.publish(pc2_msg)

    def create_pointcloud2(self, points):
        """Helper om PointCloud2 binary data te maken"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        data = []
        # Gebruik struct pack voor snelheid en correcte binary format
        for x, y, z in points:
            data.append(struct.pack('fff', x, y, z))
        
        data_binary = b"".join(data)

        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(points)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 12
        pc2_msg.row_step = pc2_msg.point_step * len(points)
        pc2_msg.is_dense = True
        pc2_msg.data = data_binary
        return pc2_msg

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        package_share_dir = get_package_share_directory('no_go_zones')
        # Zorg dat deze map naam en bestandsnaam kloppen!
        yaml_file = os.path.join(package_share_dir, 'map', 'WHEELTEC.yaml')
        
        node = DynamicObstaclePublisher(yaml_file)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"CRASH: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
