import requests
import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32

import socketio
from datetime import datetime


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
            Float32,
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
        self.sio.on('robot-manual-drive', self.robot_manual_drive)
        self.sio.on('robot-stop-manual-drive', self.robot_stop_manual_drive)

        # Connect to server
        server_ip = 'http://192.168.137.100:80'
        self.sio.connect(server_ip, retry=True)
        self.get_logger().info(f"Connected to server at {server_ip}")

        # Start thread die schedule blijft ophalen
        self.schedule_thread = threading.Thread(target=self.schedule_updater, daemon=True)
        self.schedule_thread.start()


    # ---------------- SETTINGS OPHALEN ----------------

    def schedule_updater(self):

        day_map = {
            "Mon": "M",
            "Tue": "D",
            "Wed": "W",
            "Thu": "T",
            "Fri": "F",
            "Sat": "S",
            "Sun": "U"
        }

        while True:

            try:
                response = requests.get(URL)

                if response.status_code == 200:

                    data = response.json()
                    settings = data[0]

                    schedule = settings["schedule"]

                    # Bestand pad
                    file_path = "/home/wheeltec_ros2/src/quiz_bt_node/schedule.txt"

                    # Open file in write-mode
                    with open(file_path, "w") as f:

                        for day_name, info in schedule.items():
                            prefix = day_map.get(day_name, "?")

                            if info["active"]:
                                # Verwijder ":" uit start/end en combineer
                                start = info["start"].replace(":", "")
                                end = info["end"].replace(":", "")
                                line = f"{prefix}{start}{end}"
                            else:
                                line = f"{prefix}{'X'*8}"  # 8 X'en voor inactive dag

                            f.write(line + "\n")

                    self.get_logger().info("Schedule geupdate (plain text)")

                else:
                    self.get_logger().error(f"Server fout: {response.status_code}")

            except Exception as e:
                self.get_logger().error(f"Schedule update error: {e}")

            time.sleep(INTERVAL)


    # ---------------- BATTERY ----------------

    def battery_callback(self, msg):

        percentage = msg.data
        self.get_logger().info(f'Battery percentage: {percentage}%')

        self.sio.emit("battery-percentage", {"percentage": percentage})


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


# ---------------- MAIN ----------------

def main():

    rclpy.init()

    node = QuizBTNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.sio.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
