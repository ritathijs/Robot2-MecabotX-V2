import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from std_msgs.msg import String

import os

laadcyclus_file = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/laadcyclus_status.txt"

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
        self.lastmessage_estop = past_time
        self.lastmessage_turn = past_time
        self.lastmessage_charge = past_time
        self.lastmessage_bump = past_time
        self.lastmessage_search = past_time
        self.lastmessage_nav = past_time

        self.last_cmd = Twist()
        self.active_source = "NONE"

        # Subscribers
        self.gui_sub = self.create_subscription(
            Twist,
            '/gui_cmd_vel',
            self.gui_callback,
            10)

        self.estop_sub = self.create_subscription(
            Twist,
            '/estop_cmd_vel',
            self.estop_callback,
            10)

        self.turn_sub = self.create_subscription(
            Twist,
            '/turn_cmd_vel',
            self.turn_callback,
            10)

        self.charge_sub = self.create_subscription(
            Twist,
            '/charge_cmd_vel',
            self.charge_callback,
            10)

        self.bump_sub = self.create_subscription(
            Twist,
            '/bump_cmd_vel',
            self.bump_callback,
            10)

        self.search_sub = self.create_subscription(
            Twist,
            '/search_cmd_vel',
            self.search_callback,
            10)

        self.nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.nav_callback,
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

    def gui_callback(self, msg):
        self.lastmessage_gui = self.get_clock().now()
        if 6 >= self.currentPriority():
            self.publish_command(msg, "GUI")

    def estop_callback(self, msg):
        self.lastmessage_estop = self.get_clock().now()
        if 5 >= self.currentPriority():
            self.publish_command(msg, "ESTOP")

    def turn_callback(self, msg):
        self.lastmessage_turn = self.get_clock().now()
        if 4 >= self.currentPriority():
            self.publish_command(msg, "TURN")

    def charge_callback(self, msg):
        self.lastmessage_charge = self.get_clock().now()
        if 3 >= self.currentPriority():
            self.publish_command(msg, "CHARGE")

    def bumper_ignoren(self):
        try:
            with open(laadcyclus_file, "r") as f:
                status = f.read().strip().upper()
            return status == "LAADCYCLUS"
        except Exception:
            return False

    def bump_callback(self, msg):
        self.lastmessage_bump = self.get_clock().now()
        if self.bumper_ignoren():
            return
        if 2 >= self.currentPriority():
            self.publish_command(msg, "BUMP")

    def search_callback(self, msg):
        self.lastmessage_search = self.get_clock().now()
        if 1 >= self.currentPriority():
            self.publish_command(msg, "SEARCH")

    def nav_callback(self, msg):
        self.lastmessage_nav = self.get_clock().now()
        if 0.5 >= self.currentPriority():
            self.publish_command(msg, "NAV")

    # -------------------------
    # prioriteit
    # -------------------------

    def bump_signal_recent(self):
        """Is er recent (binnen timeout) een bump-bericht binnengekomen,
        ongeacht of de bumper op dit moment effectief meetelt."""
        now = self.get_clock().now()
        return now - self.lastmessage_bump <= Duration(seconds=0.5)

    def bump_effectively_blocking(self):
        """De bumper telt enkel echt mee als er recent een bericht was
        EN IR-docking niet actief is. Dit is exact dezelfde voorwaarde
        als in currentPriority(), zodat status en gedrag altijd matchen."""
        return (not self.bumper_ignoren()) and self.bump_signal_recent()

    def currentPriority(self):
        now = self.get_clock().now()

        # 1. GUI (Hoogste)
        if now - self.lastmessage_gui <= Duration(seconds=2):
            return 6

        # 2. ESTOP
        elif now - self.lastmessage_estop <= Duration(seconds=0.5):
            return 5

        # 3. TURN
        elif now - self.lastmessage_turn <= Duration(seconds=2):
            return 4

        # 4. CHARGE
        elif now - self.lastmessage_charge <= Duration(seconds=0.5):
            return 3

        # 5. BUMP
        elif self.bump_effectively_blocking():
            return 2

        # 6. SEARCH
        elif now - self.lastmessage_search <= Duration(seconds=0.5):
            return 1

        # 7. NAV (Gewone cmd_vel)
        elif now - self.lastmessage_nav <= Duration(seconds=0.5):
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
        elif self.active_source in ["BUMP", "CHARGE"]:
            color = ORANGE
        elif self.active_source in ["NAV", "GUI", "TURN", "SEARCH"]:
            color = GREEN
        else:
            color = GRAY

        print("\033c", end="")

        print(color)
        print("========== DRIVE MUX STATUS ==========")
        print(RESET)

        print(f"{color}Actieve bron : {self.active_source}{RESET}")
        print(f"Prioriteit   : {priority}")

        # ---- IR docking / bumper status: ALTIJD tonen ----
        print("\nIR docking / bumper:")
        if self.bumper_ignoren():
            print(f"{ORANGE} IR docking : ACTIEF{RESET}")
            print(
                f"{ORANGE} Bumper     : WORDT GENEGEERD "
                f"(kijkt niet, blokkeert niet){RESET}"
            )
        else:
            print(f"{GREEN} IR docking : inactief{RESET}")
            print(
                f"{GREEN} Bumper     : normale werking "
                f"(kan blokkeren){RESET}"
            )

        print("\nSnelheid:")
        print(f" linear.x  : {self.last_cmd.linear.x:.3f} m/s")
        print(f" linear.y  : {self.last_cmd.linear.y:.3f} m/s")
        print(f" angular.z : {self.last_cmd.angular.z:.3f} rad/s")

        print("\nBronnen:")

        # Bronnenlijst in volgorde van prioriteit
        sources = [
            ("GUI", self.lastmessage_gui, 2),
            ("ESTOP", self.lastmessage_estop, 0.5),
            ("TURN", self.lastmessage_turn, 2),
            ("CHARGE", self.lastmessage_charge, 0.5),
            # BUMP wordt apart afgehandeld
            ("SEARCH", self.lastmessage_search, 0.5),
            ("NAV", self.lastmessage_nav, 0.5)
        ]

        now = self.get_clock().now()

        for name, time_msg, timeout in sources:
            # Voeg BUMP in op de juiste plek in de print-lijst (na CHARGE)
            if name == "SEARCH":
                bump_age = (now - self.lastmessage_bump).nanoseconds / 1e9
                if self.bump_effectively_blocking():
                    print(f"{GREEN}{'BUMP':<8}: ACTIEF (blokkeert) ({bump_age:.2f}s){RESET}")
                elif self.bump_signal_recent() and self.bumper_ignoren():
                    print(f"{ORANGE}{'BUMP':<8}: GENEGEERD (IR docking actief) ({bump_age:.2f}s){RESET}")
                else:
                    print(f"{GRAY}{'BUMP':<8}: uit ({bump_age:.2f}s){RESET}")

            age = (now - time_msg).nanoseconds / 1e9
            if age <= timeout:
                print(f"{GREEN}{name:<8}: ACTIEF ({age:.2f}s){RESET}")
            else:
                print(f"{GRAY}{name:<8}: uit ({age:.2f}s){RESET}")

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
