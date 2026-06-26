import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

class driveMux(Node):
    def __init__(self):
        super().__init__('drive_mux')

        past_time = self.get_clock().now() - Duration(seconds=5)
        self.lastmessage_gui = past_time
        self.lastmessage_charge = past_time
        self.lastmessage_estop = past_time
        self.lastmessage_bump = past_time
        self.lastmessage_turn = past_time
        self.lastmessage_nav = past_time

        # Subscribers
        self.gui_sub = self.create_subscription(Twist, '/gui_cmd_vel', self.gui_callback, 10)
        self.charge_sub = self.create_subscription(Twist, '/charge_cmd_vel', self.charge_callback, 10)
        self.estop_sub = self.create_subscription(Twist, '/estop_cmd_vel', self.estop_callback, 10)
        self.bump_sub = self.create_subscription(Twist, '/bump_cmd_vel', self.bump_callback, 10)
        self.turn_sub = self.create_subscription(Twist, '/turn_cmd_vel', self.turn_callback, 10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel', self.nav_callback, 10)

        self.pub = self.create_publisher(Twist, '/robot_cmd_vel', 10)

    # Callbacks
    def gui_callback(self, msg):
        self.lastmessage_gui = self.get_clock().now()
        if 6 >= self.currentPriority():
            self.pub.publish(msg)

    def charge_callback(self, msg):
        self.lastmessage_charge = self.get_clock().now()
        if 5 >= self.currentPriority():
            self.pub.publish(msg)

    def estop_callback(self, msg):
        self.lastmessage_estop = self.get_clock().now()
        if 4 >= self.currentPriority():
            self.pub.publish(msg)

    def bump_callback(self, msg):
        self.lastmessage_bump = self.get_clock().now()
        if 3 >= self.currentPriority():
            self.pub.publish(msg)

    def turn_callback(self, msg):
        self.lastmessage_turn = self.get_clock().now()
        if 2 >= self.currentPriority():
            self.pub.publish(msg)

    def nav_callback(self, msg):
        self.lastmessage_nav = self.get_clock().now()
        if 1 >= self.currentPriority():
            self.pub.publish(msg)

    # Prioriteitslogica
    def currentPriority(self):
        current_time = self.get_clock().now()
        
        if (current_time - self.lastmessage_gui) <= Duration(seconds=2):
            return 6
        elif (current_time - self.lastmessage_charge) <= Duration(seconds=0.5):
            return 5
        elif (current_time - self.lastmessage_estop) <= Duration(seconds=0.5):
            return 4
        elif (current_time - self.lastmessage_bump) <= Duration(seconds=0.5):
            return 3
        elif (current_time - self.lastmessage_turn) <= Duration(seconds=2):
            return 2
        elif (current_time - self.lastmessage_nav) <= Duration(seconds=0.5):
            return 1
        else:
            return 0

def main(args=None):
    rclpy.init(args=args)
    node = driveMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
