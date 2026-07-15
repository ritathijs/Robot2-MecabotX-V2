import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import String

# Terminal kleuren
GREEN = "\033[92m"
RED = "\033[91m"
ORANGE = "\033[93m"
RESET = "\033[0m"
GRAY = "\033[90m"

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


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
        self.lastmessage_search = past_time


        docking_qos = QoSProfile(depth=1)
        docking_qos.reliability = ReliabilityPolicy.RELIABLE
        docking_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL


        self.last_cmd = Twist()
        self.active_source = "NONE"
        self.infrared_docking_active = False


        # Subscribers

        self.docking_sub = self.create_subscription(
            String,
            '/infrared_docking_status',
            self.docking_callback,
            docking_qos
        )

        self.gui_sub = self.create_subscription(
            Twist,
            '/gui_cmd_vel',
            self.gui_callback,
            10)

        self.charge_sub = self.create_subscription(
            Twist,
            '/charge_cmd_vel',
            self.charge_callback,
            10)

        self.estop_sub = self.create_subscription(
            Twist,
            '/estop_cmd_vel',
            self.estop_callback,
            10)

        self.bump_sub = self.create_subscription(
            Twist,
            '/bump_cmd_vel',
            self.bump_callback,
            10)

        self.turn_sub = self.create_subscription(
            Twist,
            '/turn_cmd_vel',
            self.turn_callback,
            10)

        self.nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav_callback,
            10)

        self.search_sub = self.create_subscription(
            Twist,
            '/search_cmd_vel',
            self.search_callback,
            10)


        self.pub = self.create_publisher(
            Twist,
            '/robot_cmd_vel',
            10)


        self.timer = self.create_timer(
            1.0,
            self.print_status)



    # -------------------------
    # callbacks
    # -------------------------

    def publish_command(self, msg, source):

        self.last_cmd = msg
        self.active_source = source

        self.pub.publish(msg)



    def gui_callback(self,msg):

        self.lastmessage_gui = self.get_clock().now()

        if 6 >= self.currentPriority():
            self.publish_command(msg,"GUI")



    def charge_callback(self,msg):

        self.lastmessage_charge = self.get_clock().now()

        if 5 >= self.currentPriority():
            self.publish_command(msg,"CHARGE")



    def estop_callback(self,msg):

        self.lastmessage_estop = self.get_clock().now()

        if 4 >= self.currentPriority():
            self.publish_command(msg,"ESTOP")



    def bump_callback(self,msg):

        self.lastmessage_bump = self.get_clock().now()

        if self.infrared_docking_active:
            return

        if 3 >= self.currentPriority():
            self.publish_command(msg,"BUMP")



    def turn_callback(self,msg):

        self.lastmessage_turn = self.get_clock().now()

        if 2 >= self.currentPriority():
            self.publish_command(msg,"TURN")



    def nav_callback(self,msg):

        self.lastmessage_nav = self.get_clock().now()

        if 1 >= self.currentPriority():
            self.publish_command(msg,"NAV")



    def search_callback(self,msg):

        self.lastmessage_search = self.get_clock().now()

        if 0.5 >= self.currentPriority():
            self.publish_command(msg,"SEARCH")



    def docking_callback(self,msg):

        command = msg.data.strip().upper()

        if command == "DOCKING_ENABLED":

            if self.infrared_docking_active:
                return

            self.infrared_docking_active = True

            print(
                ORANGE +
                "IR docking actief: bumper blokkering UIT" +
                RESET
            )


        elif command == "DOCKING_DISABLED":

            if not self.infrared_docking_active:
                return

            self.infrared_docking_active = False

            print(
                GREEN +
                "IR docking klaar: bumper blokkering AAN" +
                RESET
            )



    # -------------------------
    # prioriteit
    # -------------------------

    def currentPriority(self):

        now = self.get_clock().now()


        if now - self.lastmessage_gui <= Duration(seconds=2):
            return 6

        elif now - self.lastmessage_charge <= Duration(seconds=0.5):
            return 5

        elif now - self.lastmessage_estop <= Duration(seconds=0.5):
            return 4

        elif (
            not self.infrared_docking_active
            and now - self.lastmessage_bump <= Duration(seconds=0.5)
        ):
            return 3

        elif now - self.lastmessage_turn <= Duration(seconds=2):
            return 2

        elif now - self.lastmessage_nav <= Duration(seconds=0.5):
            return 1

        elif now - self.lastmessage_search <= Duration(seconds=0.5):
            return 0.5

        else:
            return 0



    # -------------------------
    # terminal status
    # -------------------------

    def print_status(self):

        priority = self.currentPriority()


        if self.active_source == "ESTOP":
            color = RED

        elif self.active_source in ["BUMP","CHARGE"]:
            color = ORANGE

        elif self.active_source in ["NAV","GUI","TURN","SEARCH"]:
            color = GREEN

        else:
            color = GRAY


        print("\033c", end="")


        print(color)
        print("========== DRIVE MUX STATUS ==========")
        print(RESET)


        print(
            f"{color}Actieve bron : {self.active_source}"
            f"{RESET}")

        print(
            f"Prioriteit   : {priority}"
        )


        print("\nSnelheid:")

        print(
            f" linear.x  : {self.last_cmd.linear.x:.3f} m/s")

        print(
            f" linear.y  : {self.last_cmd.linear.y:.3f} m/s")

        print(
            f" angular.z : {self.last_cmd.angular.z:.3f} rad/s")


        print("\nBronnen:")


        sources = [
            ("GUI",self.lastmessage_gui,2),
            ("CHARGE",self.lastmessage_charge,0.5),
            ("ESTOP",self.lastmessage_estop,0.5),
            ("BUMP",self.lastmessage_bump,0.5),
            ("TURN",self.lastmessage_turn,2),
            ("NAV",self.lastmessage_nav,0.5),
            ("SEARCH",self.lastmessage_search,0.5)
        ]


        now=self.get_clock().now()


        for name,time,timeout in sources:

            age = (now-time).nanoseconds/1e9


            if age <= timeout:

                print(
                    f"{GREEN}{name:<8}: ACTIEF "
                    f"({age:.2f}s){RESET}")

            else:

                print(
                    f"{GRAY}{name:<8}: uit "
                    f"({age:.2f}s){RESET}")


        print("\n======================================")



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
