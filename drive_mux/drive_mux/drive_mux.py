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
        self.current_priority = 0

        # Subscribers
        self.gui_sub = self.create_subscription(Twist, '/gui_cmd_vel', self.gui_callback, 10)
        self.charge_sub = self.create_subscription(Twist, '/charge_cmd_vel', self.charge_callback, 10)
        self.estop_sub = self.create_subscription(Twist, '/estop_cmd_vel', self.estop_callback, 10)
        self.bump_sub = self.create_subscription(Twist, '/bump_cmd_vel', self.bump_callback, 10)
        self.turn_sub = self.create_subscription(Twist, '/turn_cmd_vel', self.turn_callback, 10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel', self.nav_callback, 10)

        self.pub = self.create_publisher(Twist, '/robot_cmd_vel', 10)

        self.timer = self.create_timer(1.0, self.logger)

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
            self.current_priority = 6
            return 6
        elif (current_time - self.lastmessage_charge) <= Duration(seconds=0.5):
            self.current_priority = 5
            return 5
        elif (current_time - self.lastmessage_estop) <= Duration(seconds=0.5):
            self.current_priority = 4
            return 4
        elif (current_time - self.lastmessage_bump) <= Duration(seconds=0.5):
            self.current_priority = 3
            return 3
        elif (current_time - self.lastmessage_turn) <= Duration(seconds=2):
            self.current_priority = 2
            return 2
        elif (current_time - self.lastmessage_nav) <= Duration(seconds=0.5):
            self.current_priority = 1
            return 1
        else:
            self.current_priority = 0
            return 0

    def logger(self):
        # ANSI Kleurcodes
        COLOR_GREEN = '\033[92m'   # Actief & wordt momenteel doorgestuurd
        COLOR_YELLOW = '\033[93m'  # Actief, maar overschreven door hogere prioriteit
        COLOR_RED = '\033[91m'     # Inactief (timeout verstreken)
        COLOR_RESET = '\033[0m'

        current_time = self.get_clock().now()
        current_prio = self.currentPriority()

        # Helper functie om de status en kleur per topic te bepalen
        def get_status(last_msg_time, timeout_sec, prio_level):
            # Controleer of het bericht binnen de timeout valt
            is_active = (current_time - last_msg_time) <= Duration(seconds=timeout_sec)
            
            if is_active and current_prio == prio_level:
                return f"{COLOR_GREEN}[DOORGESTUURD]{COLOR_RESET}"
            elif is_active:
                return f"{COLOR_YELLOW}[ACTIEF]{COLOR_RESET}"
            else:
                return f"{COLOR_RED}[INACTIEF]{COLOR_RESET}"

        # Bepaal de status voor elk topic op basis van hun specifieke timeouts en prioriteit (6 t/m 1)
        s_gui = get_status(self.lastmessage_gui, 2.0, 6)
        s_charge = get_status(self.lastmessage_charge, 0.5, 5)
        s_estop = get_status(self.lastmessage_estop, 0.5, 4)
        s_bump = get_status(self.lastmessage_bump, 0.5, 3)
        s_turn = get_status(self.lastmessage_turn, 2.0, 2)
        s_nav = get_status(self.lastmessage_nav, 0.5, 1)

        # Dashboard printen via de ROS 2 logger
        # Gebruik \n om het blok visueel te scheiden in de terminal
        self.get_logger().info(
            f"\n--- Drive Mux Status ---\n"
            f"6. /gui_cmd_vel:    {s_gui}\n"
            f"5. /charge_cmd_vel: {s_charge}\n"
            f"4. /estop_cmd_vel:  {s_estop}\n"
            f"3. /bump_cmd_vel:   {s_bump}\n"
            f"2. /turn_cmd_vel:   {s_turn}\n"
            f"1. /cmd_vel (nav):  {s_nav}\n"
            f"------------------------"
        )

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
