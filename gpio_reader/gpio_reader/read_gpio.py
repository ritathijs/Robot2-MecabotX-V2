import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time


PINS = [7, 15, 31]

class GPIOReaderNode(Node):
    def __init__(self):
        super().__init__('gpio_reader_node')
        
        # ROS publisher
        self.publisher_ = self.create_publisher(String, 'gpio_states', 10)
        
        # GPIO setup
        GPIO.setmode(GPIO.BOARD)  # BOARD om fysieke boardnummering te gebruiken
        for pin in PINS:
        # Alle pinnen in de lijst PINS initialiseren
            GPIO.setup(pin, GPIO.IN)
        
        # Timer voor periodieke uitlezing
        self.timer = self.create_timer(1.0, self.read_gpio)  # elke 1 seconde
        
	# Publiceren van data op topic
    def read_gpio(self):
        states = {pin: GPIO.input(pin) for pin in PINS}
        msg = String()
        msg.data = str(states)
        self.publisher_.publish(msg)
        self.get_logger().info(f"GPIO states: {states}")

def main(args=None):
    rclpy.init(args=args)
    node = GPIOReaderNode()
    try:
        rclpy.spin(node) # Node aanroepen (uitvoeren)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

