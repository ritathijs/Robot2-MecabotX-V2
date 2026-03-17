import requests
import json
import time
import threading
import signal
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8

import socketio

URL = "http://192.168.137.100/cms/getSettings"
INTERVAL = 300  # seconden (5 minuten)


class QuizBTNode(Node):

    def __init__(self):
        super().__init__('quiz_bt_node')

        # ROS2 publisher
        self.quiz_publisher = self.create_publisher(String, 'quiz', 10)

        # ROS2 subscriber
        self.subscription = self.create_subscription(
            String,
            'rpitopic',
            self.rpi_callback,
            10
        )

        self.battery_subscription = self.create_subscription(
            Int8,
            '/battery_percentage',
            self.battery_callback,
            10
        )

        # Socket.IO client
        self.sio = socketio.Client()
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('quiz-finished', self.on_quiz_finished)
        self.sio.on('quiz_inactive', self.on_quiz_inactive)
        self.sio.on('drive_to_quiz_location', self.on_drive_to_quiz_location)
        self.sio.on('manual-drive-start', self.robot_manual_drive)
        self.sio.on('manual-drive-stop', self.robot_stop_manual_drive)

        # Connect to server
        server_ip = 'http://192.168.137.100:80'
        self.sio.connect(server_ip, retry=True)
        self.get_logger().info(f"Connected to server at {server_ip}")

        # Flag om thread te stoppen
        self._running = True

        # Start thread die schedule blijft ophalen
        self.schedule_thread = threading.Thread(target=self.schedule_updater, daemon=True)
        self.schedule_thread.start()

    # ---------------- SETTINGS OPHALEN ----------------
    def schedule_updater(self):
        day_map = {
            "Mon": "M", "Tue": "D", "Wed": "W", "Thu": "T",
            "Fri": "F", "Sat": "S", "Sun": "U"
        }

        while self._running:
            try:
                self.get_logger().info("Fetching schedule from server...")
                response = requests.get(URL)

                if response.status_code == 200:
                    data = response.json()
                    
                    # VERANDERING: 'data' is al de dictionary, dus we hebben data[0] niet nodig
                    # Als de server soms een lijst stuurt en soms een dict, gebruik dan:
                    settings = data[0] if isinstance(data, list) else data

                    schedule = settings.get("schedule", {})
                    file_path = "/home/wheeltec/wheeltec_ros2/src/quiz_bt_node/schedule.txt"

                    with open(file_path, "w") as f:
                        for day_name, info in schedule.items():
                            prefix = day_map.get(day_name, "?")
                            
                            # Check of de dag actief is
                            if info.get("active", False):
                                # Haal ":" weg en zorg voor fallback naar "0000"
                                start = (info.get("start") or "0000").replace(":", "")
                                end = (info.get("end") or "0000").replace(":", "")
                                
                                # Opvullen tot 4 cijfers mocht de string leeg zijn
                                start = start.ljust(4, '0')[:4]
                                end = end.ljust(4, '0')[:4]
                                
                                line = f"{prefix}{start}{end}"
                            else:
                                # Inactieve dag: prefix + 8 X'en (bijv. FXXXXXXXX)
                                line = f"{prefix}{'X'*8}"

                            f.write(line + "\n")
                    
                    self.get_logger().info("Schedule succesvol geupdate.")
                else:
                    self.get_logger().error(f"Server fout: {response.status_code}")

            except Exception as e:
                self.get_logger().error(f"Schedule update error: {e}")

            # Wacht voor de volgende interval
            for _ in range(INTERVAL):
                if not self._running: break
                time.sleep(1)

    # ---------------- BATTERY ----------------
    def battery_callback(self, msg):
        percentage = msg.data
        self.get_logger().info(f'Battery percentage: {percentage}%')
        self.sio.emit("battery-update", {"percentage": percentage})

    # ---------------- TIME HANDLER ----------------
    def handle_set_hour(self, hour_value):
        self.get_logger().info(f"[PLACEHOLDER] Zet systeemtijd naar: {hour_value}")
        print(f"Zet systeemtijd naar {hour_value} (nog niet geïmplementeerd)")

    # ---------------- QUIZ PUBLISHER ----------------
    def publish_quiz_message(self, message):
        msg = String()
        msg.data = message
        for i in range(3):
            self.quiz_publisher.publish(msg)
            time.sleep(0.05)
        self.quiz_publisher.publish(msg)
        self.get_logger().info(f'Published to quiz topic: {msg.data}')

    # ---------------- SOCKET EVENTS ----------------
    def on_connect(self):
        self.get_logger().info('Connected to server')
        self.publish_quiz_message("Connected to web app")

    def on_disconnect(self):
        self.get_logger().info('Disconnected from server')
        self.publish_quiz_message("Disconnected from web app")

    def on_quiz_finished(self):
        self.get_logger().info("Quiz finished")
        self.publish_quiz_message("quiz-finished")

    def on_quiz_inactive(self):
        self.get_logger().info("Quiz inactive")
        self.publish_quiz_message("quiz_inactive")

    def on_drive_to_quiz_location(self):
        self.get_logger().info("Drive to quiz location")
        self.publish_quiz_message("drive_to_quiz_location")

    def robot_manual_drive(self):
        self.get_logger().info("Starting robot manual drive")
        self.publish_quiz_message("RobotManualDrive")

    def robot_stop_manual_drive(self):
        self.get_logger().info("Stopping robot manual drive")
        self.publish_quiz_message("RobotStopManualDrive")

    # ---------------- ROS MESSAGES ----------------
    def rpi_callback(self, msg):
        self.get_logger().info(f'Received from RPi: {msg.data}')

        if msg.data == "RobotExplore":
            self.get_logger().info("Robot is exploring")
            self.sio.emit("robot-explore")
        elif msg.data == "RobotGoToVisitors":
            self.get_logger().info("Robot will drive to visitors")
            self.sio.emit("robot-go-to-visitors")
        elif msg.data == "RobotArrivedAtVisitors":
            self.get_logger().info("Robot arrived at visitors")
            self.sio.emit("robot-arrived-at-visitors")
        elif msg.data == "robot-arrived-at-quiz-location":
            self.get_logger().info("Robot is at quiz location")
            self.sio.emit("robot-arrived-at-quiz-location")
        elif msg.data == "RobotGoCharge":
            self.get_logger().info("Robot going to charge")
            self.sio.emit("robot-go-charge")
        elif msg.data == "RobotCharging":
            self.get_logger().info("Robot is charging")
            self.sio.emit("robot-charging")
        elif msg.data == "RobotStartup":
            self.get_logger().info("Robot awaking from charging")
            self.sio.emit("robot-startup")
        elif msg.data.startswith("SETHOUR:"):
            hour_value = msg.data.split("SETHOUR:")[1].strip()
            self.handle_set_hour(hour_value)

    # ---------------- CLEANUP ----------------
    def shutdown(self):
        self.get_logger().info("Shutting down node...")
        self._running = False
        if self.schedule_thread.is_alive():
            self.schedule_thread.join()
        if self.sio.connected:
            self.sio.disconnect()
        self.destroy_node()
        rclpy.shutdown()


# ---------------- MAIN ----------------
def main():
    rclpy.init()
    node = QuizBTNode()

    # Zorg dat Ctrl+C netjes wordt afgehandeld
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
