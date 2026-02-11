import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_msgs.srv import LoadMap

class KeepoutToggler(Node):
    def __init__(self):
        super().__init__('keepout_filter_throttle')

        self.mask_full = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC_KEEPOUT_WORKING.yaml'
        self.mask_empty = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC_KEEPOUT_SERVICE.yaml'

        self.subscription = self.create_subscription(Bool, '/toggle_keepout', self.toggle_callback, 10)
        
        self.client = self.create_client(LoadMap, '/keepout_map_server/load_map')

    def toggle_callback(self, msg):

        request = LoadMap.Request()
        request.map_url = self.mask_full if msg.data else self.mask_empty
        
        self.get_logger().info(f"Verzoek versturen: {'Zone AAN' if msg.data else 'Zone UIT'}")
        self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = KeepoutToggler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
