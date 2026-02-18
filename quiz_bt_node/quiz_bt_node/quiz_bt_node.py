import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socketio


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

        # Socket.IO client
        self.sio = socketio.Client()
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('quiz-finished', self.on_quiz_finished)
        self.sio.on('quiz_inactive', self.on_quiz_inactive)
        self.sio.on('drive_to_quiz_location', self.on_drive_to_quiz_location)

        # Connect to server
        server_ip = 'http://192.168.137.100:80'
        self.sio.connect(server_ip, retry=True)
        self.get_logger().info(f"Connected to server at {server_ip}")


    def publish_quiz_message(self, message):
        msg = String()
        msg.data = message
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

