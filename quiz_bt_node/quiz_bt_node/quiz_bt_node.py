import requests
import time
import signal
import sys

import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8


# Socket.IO client voor communicatie met server
import socketio

# QoS instellingen voor ROS2 (betrouwbaarheid van berichten)
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist

# URL van server waar settings (schedule) worden opgehaald
URL = "http://192.168.137.100/cms/getSettings"


class QuizBTNode(Node):
    def __init__(self):

        super().__init__('quiz_bt_node') #aanmaak ros2node

        # ROS2 publisher

        self.blocking = False  # gebruikt om tijdelijk inkomende events te negeren
        self._admin_timer = None #voor delayed acties

        # Publisher om eventuele stuurcommando's naar robot te sturen
        self.gui_cmd_vel_publisher = self.create_publisher(Twist, '/gui_cmd_vel', 1)

        # tijdstip van het laatste commando (nodig om auto. te stoppen)
        self.last_drive_cmd_time = time.time()

        # is robot nu aan het bewegen
        self.is_moving = False

        # timer die elke 50 ms checkt of robot moet stoppen of niet
        self.check_drive_timer = self.create_timer(0.05, self.check_drive)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE

        # publisher die quiz-related-events naar BT stuurt
        self.quiz_publisher = self.create_publisher(String, 'quiz', 1)

        # publisher die admin-panel-related-events naar BT stuurt
        self.admin_publisher = self.create_publisher(String, 'admin', qos)

        # publisher die connection-related-events naar BT stuurt (robot is verbinding kwijt of niet)
        self.connection_publisher = self.create_publisher(String, 'connection', qos)

        # publisher die iets stuurt indien manual driving is gebeurd (nodig voor eventuele reset locatie)
        self.manual_drive_control_publisher = self.create_publisher(String, 'ManualDriveControleLocation', qos)

        # Berichten van BT om schermen aan te vragen
        self.subscription = self.create_subscription(
            String,
            'rpitopic',
            self.rpi_callback,
            10
        )

        # Berichten van autochargecode om batterijpercentage te krijgen
        self.battery_subscription = self.create_subscription(
            Int8,
            '/battery_percentage',
            self.battery_callback,
            10
        )

        # Socket.IO client
        self.sio = socketio.Client()

        # Koppeling van inkomende berichten aan hun callback functie
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('quiz-finished', self.on_quiz_finished)
        self.sio.on('quiz_inactive', self.on_quiz_inactive)
        self.sio.on('drive_to_quiz_location', self.on_drive_to_quiz_location)
        
        self.sio.on('admin-panel-open', self.on_admin_panel_open)
        self.sio.on('time-updated', self.on_time_updated)

        self.sio.on('admin-panel-closed', self.on_admin_panel_closed)

        self.sio.on('schedule-updated', self.on_schedule_updated)


        self.sio.on('drive-forward', lambda data=None: self.on_drive('forward'))
        self.sio.on('drive-backward', lambda data=None: self.on_drive('backward'))
        self.sio.on('drive-left', lambda data=None: self.on_drive('left'))
        self.sio.on('drive-right', lambda data=None: self.on_drive('right'))
        self.sio.on('drive-cw', lambda data=None: self.on_drive('cw'))
        self.sio.on('drive-ccw', lambda data=None: self.on_drive('ccw'))
        self.sio.on('drive-stop', lambda data=None: self.on_drive('stop'))
        
        # Connect to server
        server_ip = 'http://192.168.137.100:80'
        self.sio.connect(server_ip, retry=True)
        self.get_logger().info(f"Connected to server at {server_ip}")

    # ---------------- SETTINGS OPHALEN ----------------
    def fetch_schedule(self):
        # mapping van de dagnamen uit de server naar het 1 letter formaat dat in de schedule.txt staat
        day_map = {
            "Mon": "M", "Tue": "D", "Wed": "W", "Thu": "T",
            "Fri": "F", "Sat": "S", "Sun": "U"
        }

        try:
            self.get_logger().info("Fetching schedule from server...")
            # http get resuest naar backend om settings op te halen
            response = requests.get(URL)

            # was het request succesvol?
            if response.status_code == 200:
                data = response.json()
                # indien lijst : pak eerste element, anders het hele object (redundant)
                settings = data[0] if isinstance(data, list) else data

                # haal schedule object uit settings
                schedule = settings.get("schedule", {})

                # pad naar bestand waar schedule wordt opgeslagen
                file_path = "/home/wheeltec/wheeltec_ros2/src/quiz_bt_node/schedule.txt"

                with open(file_path, "w") as f:

                    # ittereer over elke dag in het schema
                    for day_name, info in schedule.items():

                        # zet dag om naar 1 letterige prefix
                        prefix = day_map.get(day_name, "?")

                        # haal start en eindtijd op
                        if info.get("active", False):
                            start = (info.get("start") or "0000").replace(":", "")
                            end = (info.get("end") or "0000").replace(":", "")
                            #zorg dat tijden exact 4 karakters zijn bv 1000
                            start = start.ljust(4, '0')[:4]
                            end = end.ljust(4, '0')[:4]

                            line = f"{prefix}{start}{end}"
                        else:
                            line = f"{prefix}{'X'*8}"
                        # schrijf lijn naar bestand
                        f.write(line + "\n")

                    # Schrijf ROBOTACTIVE pas **na** alle dagen
                    robot_active = settings.get("robotActive", False)
                    f.write(f"ROBOTACTIVE:{str(robot_active).lower()}\n")

                self.get_logger().info("Schedule succesvol geupdate.")
            else:  #server gaf fout
                self.get_logger().error(f"Server fout: {response.status_code}")

        except Exception as e:
            self.get_logger().error(f"Schedule update error: {e}")

    # ---------------- BATTERY ----------------
    def battery_callback(self, msg):
        percentage = msg.data
        self.get_logger().info(f'Battery percentage: {percentage}%')
        self.sio.emit("battery-update", {"percentage": percentage})


    def _is_blocking(self):
        return self.blocking

    # ---------------- QUIZ PUBLISHER ----------------
    def publish_quiz_message(self, message):
        msg = String()
        msg.data = message
        self.quiz_publisher.publish(msg)
        self.get_logger().info(f'Published to quiz topic: {msg.data}')


    def on_drive(self, direction):

        # wordt aangeroepen wanneer manual drive via GUI gebeurd

        self.get_logger().info(f'Direction received: {direction}')
        self.last_drive_cmd_time = time.time()
        self.is_moving = True

        if hasattr(self, 'admin_panel_open') and self.admin_panel_open:
            self.manual_drive_since_admin_open = True


        msg = Twist()

        if direction == 'forward':
            msg.linear.x = 0.15
            self.get_logger().info(f'0.15')
        elif direction == 'backward':
            msg.linear.x = -0.25
            self.get_logger().info(f'0.25')
        elif direction == 'left':
            msg.linear.y = -0.1
            self.get_logger().info(f'0.1')
        elif direction == 'right':
            msg.linear.y = 0.1
            self.get_logger().info(f'0.1')
        elif direction == 'cw':
            msg.angular.z = -0.2
            self.get_logger().info(f'0.2')
        elif direction == 'ccw':
            msg.angular.z = 0.2
            self.get_logger().info(f'0.2')
        else: # stop
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0

        self.gui_cmd_vel_publisher.publish(msg)

    def check_drive(self):
        # zorgt ervoor indien er geen nieuw commando komt dat robot stopt
        if self._is_blocking():
            return
        if self.is_moving and (time.time() - self.last_drive_cmd_time > 0.3):
            self.get_logger().info("Manual drive stopped")
            
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            
            self.gui_cmd_vel_publisher.publish(msg)

            self.is_moving = False

    # ---------------- ADMIN PUBLISHER ----------------
    def publish_admin_message(self, message):
        msg = String()
        msg.data = message
        self.admin_publisher.publish(msg)
        self.get_logger().info(f'Published to admin topic: {msg.data}')


    # ---------------- COONNECTION PUBLISHER ----------------

    def publish_connection_message(self, message):
        msg = String()
        msg.data = message
        self.connection_publisher.publish(msg)
        self.get_logger().info(f'Published to connection topic: {msg.data}')


    def on_time_updated(self, time_str):
        try:
            subprocess.run(["sudo", "date", "-s", time_str], check=True)
            print("Systeemtijd succesvol aangepast.")
        except subprocess.CalledProcessError as e:
            print(f"Fout bij het aanpassen van de tijd: {e}")
    # ---------------- SOCKET EVENTS ----------------
    def on_connect(self):
        if self._is_blocking():
            return
        self.get_logger().info('Connected to server')
        self.sio.emit("identification", "orin-nano-robot")
        self.publish_connection_message("CONNECT")   
        self.fetch_schedule()

    def on_disconnect(self):
        if self._is_blocking():
            return
        self.get_logger().info('Disconnected from server')
        self.publish_connection_message("DISCONNECTED")

    def on_quiz_finished(self):
        if self._is_blocking():
            return
        self.get_logger().info("Quiz finished")
        self.publish_quiz_message("quiz-finished")

    def on_quiz_inactive(self):
        if self._is_blocking():
            return
        self.get_logger().info("Quiz inactive")
        self.publish_quiz_message("quiz_inactive")

    def on_drive_to_quiz_location(self):
        if self._is_blocking():
            return
        self.get_logger().info("Drive to quiz location")
        self.publish_quiz_message("drive_to_quiz_location")

    def on_schedule_updated(self):
        if self._is_blocking():
            return
        self.get_logger().info("Schedule update event ontvangen")
        self.fetch_schedule()



    def on_admin_panel_open(self):

        if self._is_blocking():
            return

        # negeer adminpanelopen als adminpanel al open was
        if hasattr(self, 'admin_panel_open') and self.admin_panel_open:
            self.get_logger().info("Admin panel was al open -> negeren")
            return
        self.get_logger().info("Admin Panel geopend")
        self.admin_panel_open = True
        self.manual_drive_since_admin_open = False # resetten van variabele
        self.publish_admin_message("ADMINPANELOPEN")

    def on_admin_panel_closed(self):
        if self._is_blocking():
            return

        self.get_logger().info("Admin Panel gesloten")
        self.admin_panel_open = False

        # als er manual drive is gebeurd, dan kan het zijn dat robotlocatie gereset moet worden
        # hierbij moet vanalles gebeuren, zoals het loskoppelen van robot van laadstation
        # we kiezen om de ADMINPANELCLOSED 10 seconde uit te stellen om zo niet direct in BT verder te gaan
        if self.manual_drive_since_admin_open:
            self.get_logger().info("Manual drive gedetecteerd -> 10s vertraging")

            self.blocking = True

            msg = String()
            msg.data = "MANUAL_DRIVE_CONTROL"
            self.manual_drive_control_publisher.publish(msg)

            self._admin_timer = self.create_timer(10.0, self._finalize_admin_closed_wrapper)

        else:
            self.get_logger().info("Geen manual drive gedaan : meteen ADMINPANELCLOSED")

            self.publish_admin_message("ADMINPANELCLOSED")
            self.fetch_schedule()

    def _finalize_admin_closed_wrapper(self):

        # Na delay van 10 seconden  om timer mooi af te sluiten
        if self._admin_timer is not None:
            self._admin_timer.cancel()
            self._admin_timer = None

        self._finalize_admin_closed()

    def _finalize_admin_closed(self):
        self.get_logger().info("Na vertraging adminpanelclosed")

        self.publish_admin_message("ADMINPANELCLOSED")
        self.fetch_schedule() # bij sluiten adminpanel zeker de instellingen opvragen

        self.manual_drive_since_admin_open = False
        self.blocking = False

    # def _delayed_admin_closed_publish(self):
    #     time.sleep(10) 
    #     self.publish_admin_message("ADMINPANELCLOSED")
    #     self.fetch_schedule()
    #     self.manual_drive_since_admin_open = False
        
    # ---------------- ROS MESSAGES ----------------
    def rpi_callback(self, msg):
        if self._is_blocking():
            return
        self.get_logger().info(f'Received from RPi: {msg.data}')

        if msg.data == "RobotExplore":
            self.sio.emit("robot-explore")
        elif msg.data == "RobotGoToVisitors":
            self.sio.emit("robot-go-to-visitors")
        elif msg.data == "RobotArrivedAtVisitors":
            self.sio.emit("robot-arrived-at-visitors")
        elif msg.data == "robot-arrived-at-quiz-location":
            self.sio.emit("robot-arrived-at-quiz-location")
        elif msg.data == "RobotError":
            self.sio.emit("robot-error")
        elif msg.data == "RobotGoCharge":
            self.sio.emit("robot-go-charge")
        elif msg.data == "RobotCharging":
            self.sio.emit("robot-charging")
        elif msg.data == "RobotStartup":
            self.sio.emit("robot-startup")

    # ---------------- CLEANUP ----------------
    def shutdown(self):
        self.get_logger().info("Shutting down node...")
        if self.sio.connected:
            self.sio.disconnect()
        self.destroy_node()
        rclpy.shutdown()

# ---------------- MAIN ----------------
def main():
    rclpy.init()
    node = QuizBTNode()

    def signal_handler(sig, frame):
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()

if __name__ == '__main__':
    main()
