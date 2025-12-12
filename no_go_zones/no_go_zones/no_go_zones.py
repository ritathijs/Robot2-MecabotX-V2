#!/usr/bin/env python3

import os
import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

# --- PIXEL THRESHOLDS ---
THRESHOLD_WEG_MIN = 225
RANGE_EXTENDED_MIN = 175
RANGE_EXTENDED_MAX = 205

class DynamicObstaclePublisher(Node):

    def __init__(self, yaml_file):
        super().__init__('dynamic_obstacle_publisher')

        # --- Parameters ---
        self.declare_parameter('allow_extended_zone', True)
        self.allow_extended_zone = self.get_parameter('allow_extended_zone').value
        
        # --- YAML & PGM Laden ---
        try:
            with open(yaml_file, 'r') as f:
                map_yaml = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Kon YAML niet laden: {yaml_file}. Fout: {e}")
            raise e

        yaml_dir = os.path.dirname(yaml_file)
        pgm_file_absolute = os.path.join(yaml_dir, map_yaml['image'])
        self.resolution = map_yaml['resolution']
        self.origin_x, self.origin_y, _ = map_yaml['origin']

        self.get_logger().info(f"Kaart laden: {pgm_file_absolute}")
        self.img = cv2.imread(pgm_file_absolute, cv2.IMREAD_GRAYSCALE)
        
        if self.img is None:
            self.get_logger().error(f"Kon afbeelding {pgm_file_absolute} niet laden!")
            # Graceful exit in main, but here we construct nothing
            return

        self.img_height, self.img_width = self.img.shape

        # --- Publisher (Transient Local voor late-joiners) ---
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(PointCloud2, '/no_go_zones/floor_obstacles', qos_profile)

        # --- Callbacks ---
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.create_subscription(Bool, '/allow_extended_zone', self.topic_callback, 10)

        # --- Eerste publicatie ---
        self.publish_obstacles()

    # --- Callbacks ---
    def topic_callback(self, msg):
        new_state = msg.data
        if new_state != self.allow_extended_zone:
            self.get_logger().info(f"TOPIC: Schakelen naar {new_state}.")
            # Update interne variabele
            self.allow_extended_zone = new_state
            # Publiceer update
            self.publish_obstacles()
            # Probeer parameter te syncen (zonder error als node afsluit)
            try:
                self.set_parameters([Parameter('allow_extended_zone', Parameter.Type.BOOL, new_state)])
            except Exception:
                pass

    def parameter_callback(self, params):
        should_republish = False
        for param in params:
            if param.name == 'allow_extended_zone':
                if self.allow_extended_zone != param.value:
                    self.allow_extended_zone = param.value
                    self.get_logger().info(f"PARAM: Zone gewijzigd naar {self.allow_extended_zone}")
                    should_republish = True
        
        if should_republish:
            # We roepen publish niet direct aan hier, maar plannen het in of doen het na return
            # Echter, in Python ROS2 is het vaak veilig om direct te doen als de berekening snel is.
            self.publish_obstacles()
            
        return SetParametersResult(successful=True)

    # --- Core Logica (Geoptimaliseerd) ---
    def publish_obstacles(self):
        """Genereer voxel pointcloud met NumPy vectorisatie (veel sneller)."""
        
        # 1. Maak Masker
        # Alles is initieel obstakel (255)
        mask = np.full(self.img.shape, 255, dtype=np.uint8)
        
        # Witte gebieden zijn vrij (0)
        mask[self.img >= THRESHOLD_WEG_MIN] = 0
        
        # Extended zone logica
        if self.allow_extended_zone:
            mask[(self.img >= RANGE_EXTENDED_MIN) & (self.img <= RANGE_EXTENDED_MAX)] = 0

        # 2. Vind pixel coördinaten van obstakels (waar mask nog steeds 255 is)
        # y_indices (rijen), x_indices (kolommen)
        y_indices, x_indices = np.where(mask == 255)
        
        if len(x_indices) == 0:
            self.pub.publish(self.create_pointcloud2(np.array([])))
            return

        # 3. Conversie naar Wereld Coördinaten (Vectorized)
        # Formule: origin + (index * resolutie)
        # We voegen 0.5 * res toe om het punt in het midden van de pixel te centreren
        z_base = 0.05
        voxel_height = 0.05
        num_voxels_z = 4
        
        # Bereken X en Y arrays
        world_x = self.origin_x + (x_indices * self.resolution) + (self.resolution / 2)
        # Let op de Y-flip: (height - y - 1)
        world_y = self.origin_y + ((self.img_height - y_indices - 1) * self.resolution) + (self.resolution / 2)

        # 4. Creëer voxels in de hoogte (Z-as)
        # We herhalen de X en Y arrays voor elke Z laag
        all_x = np.tile(world_x, num_voxels_z)
        all_y = np.tile(world_y, num_voxels_z)
        
        # Maak Z array: [0.05, 0.05... 0.10, 0.10...]
        z_levels = [z_base + i * voxel_height for i in range(num_voxels_z)]
        all_z = np.repeat(z_levels, len(world_x)) # Repeat elk level N keer

        # 5. Samenvoegen tot PointCloud2 data structuur
        # We gebruiken een structured array voor snelheid
        cloud_data = np.zeros(len(all_x), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
        cloud_data['x'] = all_x
        cloud_data['y'] = all_y
        cloud_data['z'] = all_z

        # Publiceren
        pc2_msg = self.create_pointcloud2(cloud_data)
        self.pub.publish(pc2_msg)
        self.get_logger().info(f"Gepubliceerd: {len(all_x)} voxelpunten")

    def create_pointcloud2(self, cloud_data):
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        if len(cloud_data) == 0:
            return PointCloud2(header=header)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        pc2_msg = PointCloud2()
        pc2_msg.header = header
        pc2_msg.height = 1
        pc2_msg.width = len(cloud_data)
        pc2_msg.fields = fields
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 12  # 3 * float32 (4 bytes)
        pc2_msg.row_step = pc2_msg.point_step * len(cloud_data)
        pc2_msg.is_dense = True
        # Hier is de magie: NumPy array direct naar bytes converteren
        pc2_msg.data = cloud_data.tobytes()
        
        return pc2_msg

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        package_share_dir = get_package_share_directory('no_go_zones')
        # Zorg dat dit pad klopt in je systeem
        yaml_file = os.path.join(package_share_dir, 'map', 'WHEELTEC.yaml')
        
        node = DynamicObstaclePublisher(yaml_file)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
