import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socketio
import time




class PunisherNode(Node):
    def __init__(self):
        super().__init__('PunisherNode')
        
        # Create ROS 2 subscriber
        self.subscription = self.create_subscription(
            String,
            'rpitopic',
            self.rpi_callback,
            10
        )
        
        # Initialize socket.io client

        # Hier moet in toekomst nog check worden toegevoegd of verbinding succesvol kan worden gemaakt (en wordt onderhouden)
        print("Proberen om verbinding op te zetten")
       
        self.sio = socketio.Client()
        self.sio.connect('http://192.168.137.199:80', retry=True)
       
        print("Verbinding succesvol")

    def rpi_callback(self, msg):
        """Callback function for messages received from the Raspberry Pi topic"""
        
        self.get_logger().info(f'Received from RPi: {msg.data}')
        
        if msg.data == "RobotExplore":
            print("robot is exploring")
            self.sio.emit("robot-explore")
        elif msg.data == "RobotGoToVisitors":
            print("Path has been created, robot will be driving")
            self.sio.emit("robot-go-to-visitors")
        elif msg.data == ("RobotArrivedAtVisitors"):
            print("RobotArrivedAtVisistors")

            self.sio.emit("robot-arrived-at-visitors")
            
        elif msg.data == ("robot-arrived-at-quiz-location"):
            print("Robot is at quiz location")
            self.sio.emit("robot-arrived-at-quiz-location")
            
        



def main():
    rclpy.init()
    
    punisher_node = PunisherNode()
    
    try:
          # Keep ROS 2 node running
        rclpy.spin(punisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        
        punisher_node.sio.disconnect()
       
        punisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

