import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socketio
import time
from std_msgs.msg import Float32
from datetime import datetime


class QuizBTNode(Node):
    def __init__(self):
        super().__init__('quiz_bt_node')

        # ROS 2 Publisher voor quiz status
        self.quiz_publisher = self.create_publisher(String, 'quiz', 10)

        # ROS 2 Subscriber voor RPi commands
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



    def battery_callback(self, msg):
        percentage = msg.data
        self.get_logger().info(f'Battery percentage: {percentage}%')

        # Stuur naar webapp via socket.io
        self.sio.emit("battery-percentage", {"percentage": percentage})



    def handle_set_hour(self, hour_value):
        self.get_logger().info(f"[PLACEHOLDER] Zet systeemtijd naar: {hour_value}")
        print(f"Zet systeemtijd naar {hour_value} (nog niet geïmplementeerd)")
        # TODO: echte systeemtijd aanpassen indien nodig

    def handle_start_work_hour(self, hour_value):
        self.get_logger().info(f"[PLACEHOLDER] Werkuren starten om: {hour_value}")
        print(f"Werkuren starten om {hour_value} (placeholder)")
        # TODO: logica toevoegen voor start werkmodus

    def handle_stop_work_hour(self, hour_value):
        self.get_logger().info(f"[PLACEHOLDER] Werkuren stoppen om: {hour_value}")
        print(f"Werkuren stoppen om {hour_value} (placeholder)")
        # TODO: logica toevoegen voor stop werkmodus


    def publish_quiz_message(self, message):
        msg = String()
        msg.data = message

        for i in range(3):
            self.quiz_publisher.publish(msg)
            time.sleep(0.05)  # 50 ms vertraging

        self.quiz_publisher.publish(msg)
        self.get_logger().info(f'Published to quiz topic: {msg.data}')

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


    # ZENDDEEL : ONDERSTAANDEN WORDEN DOOR DE BT VERZONDEN RICHTING DE QUIZ

    def rpi_callback(self, msg):
        self.get_logger().info(f'Received from RPi: {msg.data}')
        
	# BEHAVIOR TREE NODE : RobotExplore
        if msg.data == "RobotExplore":
            self.get_logger().info("Robot is exploring")
            self.sio.emit("robot-explore")

	# BEHAVIOR TREE NODE : StartDrivingToPeople
        elif msg.data == "RobotGoToVisitors":
            self.get_logger().info("Robot will drive to visitors")
            self.sio.emit("robot-go-to-visitors")

	# BEHAVIOR TREE NODE : ArrivedAtVisitors
        elif msg.data == "RobotArrivedAtVisitors":
            self.get_logger().info("Robot arrived at visitors")
            self.sio.emit("robot-arrived-at-visitors")

	# BEHAVIOR TREE NODE : RobotAtQuiz
        elif msg.data == "robot-arrived-at-quiz-location":
            self.get_logger().info("Robot is at quiz location")
            self.sio.emit("robot-arrived-at-quiz-location")

	# BEHAVIOR TREE NODE : DriveToChargingStation
        elif msg.data == "RobotGoCharge":
            self.get_logger().info("Robot going to charge")
            self.sio.emit("robot-go-charge")


	# BEHAVIOR TREE NODE : IsBatteryFull
        elif msg.data == "RobotCharging":
            self.get_logger().info("Robot is charging")
            self.sio.emit("robot-charging")




        #onderstaande nog af te stemmen met Quinten
       	# BEHAVIOR TREE NODE : BatteryCharged
        elif msg.data == "RobotStartup":
            self.get_logger().info("Robot awaking from charging")
            self.sio.emit("robot-startup")

    
        elif msg.data.startswith("SETHOUR:"):
            hour_value = msg.data.split("SETHOUR:")[1].strip()
            self.handle_set_hour(hour_value)
            return

        elif msg.data.startswith("STARTWORKHOUR:"):
            hour_value = msg.data.split("STARTWORKHOUR:")[1].strip()
            self.handle_start_work_hour(hour_value)
            return

        elif msg.data.startswith("STOPWORKHOUR:"):
            hour_value = msg.data.split("STOPWORKHOUR:")[1].strip()
            self.handle_stop_work_hour(hour_value)
            return



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

