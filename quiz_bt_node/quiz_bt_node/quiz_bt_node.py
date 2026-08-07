import requests
import time
import signal
import sys
import os

import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Float32

from ament_index_python.packages import get_package_share_directory
import json

# Socket.IO client voor communicatie met server
import socketio

# QoS instellingen voor ROS2 (betrouwbaarheid van berichten)
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from socketio.exceptions import BadNamespaceError


# Basis serveradres
SERVER_IP = "10.0.0.11"

# Afgeleide URL's
URL = f"http://{SERVER_IP}/robot-status/get-robot-status"
SERVER_URL = f"http://{SERVER_IP}:80"



import requests

SERVER_IP = "10.0.0.11"


def update_robot_status(fields_to_update):
    try:
        response = requests.post(
            f"http://{SERVER_IP}/robot-status/insert-robot-status",
            json=fields_to_update,
            timeout=5
        )

        if response.status_code == 200:
            body = response.json()

            if body.get("succes") is True:
                return True

        print(f"Update robot status failed: {response.status_code}")
        print(response.text)

    except Exception as e:
        print(f"Update robot status failed: {e}")

    return False

def retrieve_robot_status(fields):
    try:
        params = {
            "fields": ",".join(fields)
        }

        response = requests.get(
            f"http://{SERVER_IP}/robot-status/get-robot-status",
            params=params,
            timeout=5
        )

        if response.status_code == 200:
            body = response.json()

            if body.get("succes") is True:
                return body.get("data", {})

        print(f"Retrieve robot status failed: {response.status_code}")
        print(response.text)

    except Exception as e:
        print(f"Retrieve robot status failed: {e}")

    return {}
# De systeemtijd van de pi wordt doorgestuurd naar de robot
# Indien volgende parameter op False staat wordt de robot zijn systeemtijd NIET aangepast
# Indien volgende parameter op True  staat wordt de robot zijn systeemtijd WEL  aangepast (kan transformproblemen leveren in RVIZ)
CHANGETIME = True

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

        self.robot_ready_sent = False

        self.manual_drive_since_admin_open = False
        self.manual_drive_db_updated = False
        
        self.ignore_screen_requests = False

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

        # Logica om bij te houden of laatst verstuurde event robot-charge is om bij adminpanelclosed juist bericht te sturen
        self.last_emitted_event = None

        self.last_screen = None


        self.quiz_location_suffix = ""

        # Berichten van BT om schermen te sturen
        self.subscription = self.create_subscription(
            String,
            'rpitopic',
            self.rpi_callback,
            1
        )

        # Berichten van autochargecode om batterijpercentage te krijgen
        self.battery_level = None
        self.battery_subscription = self.create_subscription(
            Float32,
            '/BatteryAverageVoltage',
            self.battery_callback,
            10
        )

       
        self.bump_status_subscription = self.create_subscription(
            String,
            '/bump_status',
            self.bump_status_callback,
            10
        )
        
        # publisher voor active button events
        self.ask_button_quiz_publisher = self.create_publisher(
            String,
            'ask_button_quiz',
            1
        )


        # Publisher voor noodstop
        self.estop_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/estop_cmd_vel',
            10
        )

        # timer die periodiek 0-snelheid stuurt op /estop_cmd_vel gedurende robot-stop-for-x-time
        self.estop_timer = None
        # tijdstip (time.time()) waarop de estop-periode moet eindigen
        self.estop_end_time = None



        self.quiz_activestatus_publisher = self.create_publisher(String, 'quizbtnode_activestatus', 1)


        # Socket.IO client
        self.sio = socketio.Client()

        # Koppeling van inkomende berichten aan hun callback functie
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('quiz-finished', self.on_quiz_finished)
        self.sio.on('quiz_inactive', self.on_quiz_inactive)
        self.sio.on('drive_to_quiz_location', self.on_drive_to_quiz_location)

        self.sio.on('robot-askScreen', self.on_ask_screen)

        self.sio.on('robot-askIsActive', self.on_ask_is_active)
        self.sio.on('robot-activeButtonToggled', self.on_active_button_toggled)

        self.sio.on('robot-stop-for-x-time', self.on_robot_stop_for_x_time)
        
        self.sio.on('admin-panel-open', self.on_admin_panel_open)
        self.sio.on('time-updated', self.on_time_updated)

        self.sio.on('admin-panel-closed', self.on_admin_panel_closed)

        self.sio.on('schedule-updated', self.on_schedule_updated)
        
        self.sio.on('robot-get-battery-percentage', self.robot_get_battery_percentage)

        self.sio.on('drive', self.on_drive)

        # Connect to server
        self.sio.connect(SERVER_URL, retry=True)
        self.get_logger().info(f"Connected to server at {SERVER_URL}")

        # Lees de JSON uit met de batterijniveaus om in de UI de juiste kleur te tonen (groen, oranje, rood)
        try:
            package_share_dir = get_package_share_directory('mecabot_bt')
            json_file_path = os.path.join(package_share_dir, 'trees', 'spanningsniveaus.json')
            with open(json_file_path, 'r') as json_file:
                self.spanningsniveaus = json.load(json_file)
                self.battery_low_voltage = self.spanningsniveaus.get("battery_low_voltage", 21.0)
                self.battery_high_voltage = self.spanningsniveaus.get("battery_ok_voltage", 23.0)
            self.get_logger().info(f"JSON succesvol geladen. Inhoud: {self.spanningsniveaus}")
        except Exception as e:
            self.battery_low_voltage = 21.0
            self.battery_high_voltage = 23.0
            self.get_logger().error(f"Fout bij inladen json: {e}")

        # ---------------- OPSTART: EENMALIG NAAR DATABASE SCHRIJVEN ----------------
        # Dit gebeurt ENKEL bij opstart van deze hele code (dus 1x, hier in __init__,
        # en nergens anders in de node). We zetten robotActive op true in de database.
        self.get_logger().info("Opstart: robotActive=true wegschrijven naar database...")
        if update_robot_status({"robotActive": True}):
            self.get_logger().info("Opstart: robotActive succesvol op true gezet in database.")
        else:
            self.get_logger().error("Opstart: kon robotActive niet op true zetten in database.")

    # ---------------- SETTINGS OPHALEN ----------------

    def publish_ask_button_quiz_message(self, message):
        msg = String()
        msg.data = message
        self.ask_button_quiz_publisher.publish(msg)
        self.get_logger().info(
            f'Published to ask_button_quiz topic: {msg.data}'
        )
    
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
                #settings = data[0] if isinstance(data, list) else data

                # haal schedule object uit settings
                #schedule = settings.get("schedule", {})

                # pad naar bestand waar schedule wordt opgeslagen
                file_path = "/home/wheeltec/wheeltec_ros2/src/quiz_bt_node/schedule.txt"

                with open(file_path, "w") as f:

                    # # ittereer over elke dag in het schema
                    # for day_name, info in schedule.items():

                    #     # zet dag om naar 1 letterige prefix
                    #     prefix = day_map.get(day_name, "?")

                    #     # haal start en eindtijd op
                    #     if info.get("active", False):
                    #         start = (info.get("start") or "0000").replace(":", "")
                    #         end = (info.get("end") or "0000").replace(":", "")
                    #         #zorg dat tijden exact 4 karakters zijn bv 1000
                    #         start = start.ljust(4, '0')[:4]
                    #         end = end.ljust(4, '0')[:4]

                    #         line = f"{prefix}{start}{end}"
                    #     else:
                    #         line = f"{prefix}{'X'*8}"
                    #     # schrijf lijn naar bestand
                    #     f.write(line + "\n")

                    # Schrijf ROBOTACTIVE pas **na** alle dagen
                    robot_active = settings.get("robotActive", False)
                    f.write(f"ROBOTACTIVE:{str(robot_active).lower()}\n")

                self.get_logger().info("Schedule succesvol geupdate.")
            else:  #server gaf fout
                self.get_logger().error(f"Server fout: {response.status_code}")

        except Exception as e:
            self.get_logger().error(f"Schedule update error: {e}")


    def safe_emit(self, event, data=None):
        try:
            # Alleen bij scherm-events robot-ready versturen
            screen_events = {
                "robot-explore",
                "robot-go-to-visitors",
                "robot-arrived-at-visitors",
                "robot-arrived-at-quiz-location",
                "robot-error-drive",
                "follow-robot-screen",
                "robot-error-charge",
                "robot-go-charge",
                "robot-charging",
                "robot-startup",
                "robot-docking",
                "robot-lost-charging",
            }

            if (
                not self.robot_ready_sent
                and event in screen_events
            ):
                self.sio.emit("robot-ready")
                self.robot_ready_sent = True
                self.get_logger().info("robot-ready verstuurd")

            self.sio.emit(event, data)

        except BadNamespaceError:
            self.get_logger().warn(
                f"Dropped event '{event}', socket not connected."
            )
        except Exception as e:
            self.get_logger().error(str(e))


    def _is_blocking(self):
        return self.blocking

    # ---------------- QUIZ PUBLISHER ----------------
    def publish_quiz_message(self, message):
        msg = String()
        msg.data = message
        self.quiz_publisher.publish(msg)
        self.get_logger().info(f'Published to quiz topic: {msg.data}')
        
       
    def publish_quiz_activestatus_message(self, message):
        msg = String()
        msg.data = message
        self.quiz_activestatus_publisher.publish(msg)
        self.get_logger().info(
            f'Published to quizbtnode-activestatus topic: {msg.data}'
        )


    def on_drive(self, data):
        # Controleer of er daadwerkelijk data is binnengekomen
        if not data:
            return
            
        self.last_drive_cmd_time = time.time()
        self.is_moving = True

        if hasattr(self, 'admin_panel_open') and self.admin_panel_open:
            self.manual_drive_since_admin_open = True

        direction = data.get('direction', 'stop')
        speed = float(data.get('speed', 0.0))
        
        self.get_logger().info(f'Direction received: {direction}, speed {speed}')
    
        msg = Twist()

        if hasattr(self, 'admin_panel_open') and self.admin_panel_open:
            self.manual_drive_since_admin_open = True

            if not self.manual_drive_db_updated:
                update_robot_status({
                    "manualDrive": True
                })

                self.manual_drive_db_updated = True


        if direction == 'forward':
            msg.linear.x = 0.15*speed
            self.get_logger().info(f'{0.15*speed}')
        elif direction == 'backward':
            msg.linear.x = -0.15*speed
            self.get_logger().info(f'{0.15*speed}')
        elif direction == 'left':
            msg.linear.y = -0.15*speed
            self.get_logger().info(f'{0.15*speed}')
        elif direction == 'right':
            msg.linear.y = 0.15*speed
            self.get_logger().info(f'{0.15*speed}')
        elif direction == 'cw':
            msg.angular.z = -0.15*speed
            self.get_logger().info(f'{0.15*speed}')
        elif direction == 'ccw':
            msg.angular.z = 0.15*speed
            self.get_logger().info(f'{0.15*speed}')
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
            
            
    # ---------------- BATTERY UPDATE -----------------
    
    def robot_get_battery_percentage(self, message=None):
        
        if self.battery_level != None:
            battery_val = round(self.battery_level, 2)
        else:
            battery_val = None
            
        self.safe_emit('robot-update-battery-percentage', {
            "battery": battery_val,
            "batteryLow": self.battery_low_voltage,   # Threshold voor oranje
            "batteryHigh": self.battery_high_voltage  # Threshold voor groen
        })
          
        self.get_logger().info(f"Batterij doorgestuurd: {getattr(self, 'battery_level', 0)}Volts")
    
    def battery_callback(self, msg):
        self.battery_level = msg.data
    
    # ---------------- BUMPER PUBLISHER ----------------
    def bump_status_callback(self, status):
        #self.get_logger().info(f'bumperstatus: {status.data}')
        self.safe_emit('robot-bumper-status', {
            "msg": status.data
        })

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
        if not(CHANGETIME):
            return
        #try:
         # zet in instellingen automatisch dag en tijdsbepaling over internet uit indien je de tijd wilt zien updaten
            # subprocess.run(["sudo", "date", "-s", time_str], check=True)
            #print("Systeemtijd succesvol aangepast.")
        #except subprocess.CalledProcessError as e:
            #print(f"Fout bij het aanpassen van de tijd: {e}")

    # ---------------- SOCKET EVENTS ----------------
    def on_connect(self):
        if self._is_blocking():
            return
        self.get_logger().info('Connected to server')
        self.safe_emit("identification", "orin-nano-robot")
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

    def on_ask_screen(self, data=None):
        self.get_logger().info("Scherm aangevraagd")

        if self.ignore_screen_requests:
            self.get_logger().info(
                "robot-askScreen genegeerd wegens manual-drive cooldown"
            )
            return


        # Schermen die niet opnieuw mogen worden verstuurd
        blocked_screens = {
            "robot-error-drive",
            "robot-error-charge",
            "robot-go-charge",
            "robot-charging",
            "robot-docking",
            "robot-lost-charging",
        }

        if self.last_screen in blocked_screens:
            self.get_logger().info(
                f"Laatste scherm ({self.last_screen}) mag niet opnieuw verstuurd worden"
            )
            return

        if self.last_screen:
            self.get_logger().info(
                f"Laatste scherm opnieuw versturen: {self.last_screen}"
            )
            self.safe_emit(self.last_screen)
        else:
            self.get_logger().warn("Nog geen last_screen beschikbaar")


    def on_ask_is_active(self, data=None):
        self.get_logger().info("vraag aan bt of robot actief is")
        self.publish_quiz_activestatus_message("ask-is-active")

    def on_active_button_toggled(self, is_active):
        self.get_logger().info(
            f"Gebruiker heeft op de knop gedrukt (gewenst: {is_active})"
        )

        if is_active:
            self.publish_ask_button_quiz_message(
                "robot-activeButtonToggled-true"
            )
        else:
            self.publish_ask_button_quiz_message(
                "robot-activeButtonToggled-false"
            )

    def on_robot_stop_for_x_time(self, data):
        self.get_logger().info("Hier zou de robot moeten stoppen voor max 30 seconden. Waarschijnlijk wordt tijdens 30 seconden quiz getrigger of adminpaneel geopend")

        if not data:
            self.get_logger().warn("robot-stop-for-x-time zonder data ontvangen -> genegeerd")
            return

        stop_time = data.get('time')
        self.get_logger().info(f"data.time ({stop_time} milliseconden) kan hiervoor gebruikt worden.")

        try:
            stop_time_sec = float(stop_time) / 1000.0
        except (TypeError, ValueError):
            self.get_logger().error(f"Ongeldige tijd meegegeven voor robot-stop-for-x-time: {stop_time}")
            return

        if stop_time_sec <= 0:
            self.get_logger().warn("Tijd voor robot-stop-for-x-time is 0 of negatief -> genegeerd")
            return

        # indien er al een estop-periode bezig is, stoppen we die en starten we opnieuw met de nieuwe tijd
        if self.estop_timer is not None:
            self.estop_timer.cancel()
            self.estop_timer = None

        self.estop_end_time = time.time() + stop_time_sec

        self.get_logger().info(
            f"Estop gestart: {stop_time_sec}s lang wordt 0-snelheid gestuurd op /estop_cmd_vel"
        )

        # timer die elke 50ms een 0-snelheid stuurt op /estop_cmd_vel, tot de tijd verstreken is
        self.estop_timer = self.create_timer(0.05, self._publish_estop_cmd_vel)

        # meteen 1 keer sturen zodat er niet gewacht moet worden op de eerste timer-tick
        self._publish_estop_cmd_vel()

    def _publish_estop_cmd_vel(self):
        # als de periode voorbij is: timer stoppen en niet meer sturen
        if self.estop_end_time is None or time.time() >= self.estop_end_time:
            if self.estop_timer is not None:
                self.estop_timer.cancel()
                self.estop_timer = None
            self.estop_end_time = None
            self.get_logger().info("Estop-periode afgelopen, stoppen met sturen op /estop_cmd_vel")
            return

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.estop_cmd_vel_publisher.publish(msg)

    def _stop_estop(self, reason=""):
        # Stopt een eventueel lopende estop-periode voortijdig (bv. omdat er een scherm werd aangevraagd via rpitopic)
        if self.estop_timer is not None or self.estop_end_time is not None:
            if self.estop_timer is not None:
                self.estop_timer.cancel()
                self.estop_timer = None
            self.estop_end_time = None
            self.get_logger().info(
                f"Estop voortijdig gestopt{f' ({reason})' if reason else ''}"
            )

    def on_quiz_inactive(self):
        if self._is_blocking():
            return
        self.get_logger().info("Quiz inactive")
        self.publish_quiz_message("quiz_inactive")

    def on_drive_to_quiz_location(self):
        if self._is_blocking():
            return

        event = "drive_to_quiz_location"

        if self.quiz_location_suffix:
            event += self.quiz_location_suffix
            self.get_logger().info(
                f"Drive to quiz location met suffix: {self.quiz_location_suffix}"
            )

            # reset zodat volgende keer weer normaal begint
            self.quiz_location_suffix = ""

        self.publish_quiz_message(event)

        
    def on_schedule_updated(self):
        if self._is_blocking():
            return
        self.get_logger().info("Schedule update event ontvangen")
        self.fetch_schedule()

    def emit_event(self, event, data=None):
        self.safe_emit(event, data) if data else self.safe_emit(event)
        self.last_emitted_event = event

    def on_admin_panel_open(self):

        if self._is_blocking():
            return

        # negeer adminpanelopen als adminpanel al open was
        if hasattr(self, 'admin_panel_open') and self.admin_panel_open:
            self.get_logger().info("Admin panel was al open -> negeren")
            return

        self.manual_drive_since_admin_open = False
        self.manual_drive_db_updated = False

        self.get_logger().info("Admin Panel geopend")
        self.admin_panel_open = True
        self.manual_drive_since_admin_open = False # resetten van variabele
        self.publish_admin_message("ADMINPANELOPEN")

    def on_admin_panel_closed(self):
        if self._is_blocking():
            return

        self.get_logger().info("Admin Panel gesloten")
        self.admin_panel_open = False

        if self.last_emitted_event == "robot-charging" or self.last_emitted_event == "robot-go-charge" or self.last_emitted_event == "robot-docking":
            self.get_logger().info("Laatste event was robot-charging -> opnieuw versturen")
            self.emit_event("robot-charging")
        else:
            self.get_logger().info("Laatste event was niet robot-charging -> niets versturen")

        # als er manual drive is gebeurd, dan kan het zijn dat robotlocatie gereset moet worden
        # hierbij moet vanalles gebeuren, zoals het loskoppelen van robot van laadstation
        # we kiezen om de ADMINPANELCLOSED 10 seconde uit te stellen om zo niet direct in BT verder te gaan
        if self.manual_drive_since_admin_open:
            self.get_logger().info("Manual drive gedetecteerd -> 10s vertraging")

            self.blocking = True
            self.ignore_screen_requests = True

            msg = String()
            msg.data = "MANUAL_DRIVE_CONTROL"
            self.manual_drive_control_publisher.publish(msg)

            self._admin_timer = self.create_timer(4.0, self._finalize_admin_closed_wrapper)

        else:
            self.get_logger().info("Geen manual drive gedaan : meteen ADMINPANELCLOSED")
            
            if self.last_emitted_event == "robot-charging":
                self.get_logger().info("Laatste event was robot-charging -> opnieuw versturen")
                self.emit_event("robot-charging")
            else:
                self.get_logger().info("Laatste event was niet robot-charging -> niets versturen")



    def _finalize_admin_closed_wrapper(self):

        # Na delay van 10 seconden  om timer mooi af te sluiten
        if self._admin_timer is not None:
            self._admin_timer.cancel()
            self._admin_timer = None

        self._finalize_admin_closed()

    def _finalize_admin_closed(self):
        self.get_logger().info("Na vertraging adminpanelclosed")


        self.fetch_schedule() # bij sluiten adminpanel zeker de instellingen opvragen

        update_robot_status({
            "manualDrive": False
        })

        self.manual_drive_since_admin_open = False
        self.blocking = False
        self.ignore_screen_requests = False

    # def _delayed_admin_closed_publish(self):
    #     time.sleep(10) 
    #     self.publish_admin_message("ADMINPANELCLOSED")
    #     self.fetch_schedule()
    #     self.manual_drive_since_admin_open = False
        
    # ---------------- ROS MESSAGES ----------------
    def rpi_callback(self, msg):
        if self._is_blocking():
            return

        # Zodra er een scherm wordt aangevraagd via rpitopic, stoppen we een eventueel
        # lopende estop-periode: er wordt dan niet langer 0-snelheid op /estop_cmd_vel gestuurd.
        self._stop_estop("scherm aangevraagd via rpitopic")

        self.get_logger().info(f'Received from RPi: {msg.data}')

        if msg.data == "RobotExplore":
            self.last_screen = "robot-explore"
            self.safe_emit("robot-explore")
        elif msg.data == "RobotGoToVisitors":
            self.last_screen = "robot-go-to-visitors"
            self.safe_emit("robot-go-to-visitors")
            
        elif msg.data == "RobotIsActiveTrue":
            self.safe_emit("robot-isActive", True)

        elif msg.data == "RobotIsActiveFalse":
            self.safe_emit("robot-isActive", False)

        elif msg.data.startswith("RobotArrivedAtVisitors"):
            suffix = msg.data.replace("RobotArrivedAtVisitors", "")

            # Controleer of suffix formaat A8 is (1 hoofdletter + 1 cijfer)
            if len(suffix) == 2 and suffix[0].isupper() and suffix[1].isdigit():
                self.quiz_location_suffix = suffix
                self.get_logger().info(
                    f"Quiz locatie suffix opgeslagen: {self.quiz_location_suffix}"
                )

            self.last_screen = "robot-arrived-at-visitors"
            self.safe_emit("robot-arrived-at-visitors")


        elif msg.data == "robot-arrived-at-quiz-location":
            self.last_screen = "robot-arrived-at-quiz-location"
            self.safe_emit("robot-arrived-at-quiz-location")


        elif msg.data == "RobotError":
            self.last_screen = "robot-error-drive"
            self.safe_emit("robot-error-drive")

        elif msg.data == "FollowRobotScreen":
            self.last_screen = "follow-robot-screen"
            self.safe_emit("follow-robot-screen")

        elif msg.data == "RobotErrorCharge":
            self.last_screen = "robot-error-charge"
            self.safe_emit("robot-error-charge")

        elif msg.data == "RobotGoCharge":
            self.last_screen = "robot-go-charge"
            self.safe_emit("robot-go-charge")

        elif msg.data == "RobotCharging":
            self.last_screen = "robot-charging"
            self.safe_emit("robot-charging")

        elif msg.data == "RobotStarting":
            self.last_screen = "robot-startup"
            self.safe_emit("robot-startup")

        elif msg.data == "RobotDocking":
            self.last_screen = "robot-docking"
            self.safe_emit("robot-docking")

        elif msg.data == "RobotStartup":
            self.last_screen = "robot-startup"
            self.safe_emit("robot-startup")

        elif msg.data == "RobotFailedDriveToCharging":
            self.last_screen = "robot-lost-charging"
            self.safe_emit("robot-lost-charging")

    # ---------------- CLEANUP ----------------
    def shutdown(self):
        self.get_logger().info("Shutting down node...")
        if self.estop_timer is not None:
            self.estop_timer.cancel()
            self.estop_timer = None
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
