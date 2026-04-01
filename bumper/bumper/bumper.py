import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from geometry_msgs.msg import Twist

PINS = [7, 15, 31, 32]
DIR = {"A": 15, "L": 32, "V": 7, "R": 31}  # Definieer Voor, Achter, Links, Rechts
speed = 0.1

class GPIOReaderNode(Node):
    def __init__(self):
        super().__init__('gpio_reader_node')
        
        self.bumper_cmd_vel_pub = self.create_publisher(Twist, '/bump_cmd_vel', 1)
        
        self.action = "Free"
        
        self.timer_10s = None
        self.timer_3s = None

        GPIO.setmode(GPIO.BOARD)  
        for pin in PINS:
            GPIO.setup(pin, GPIO.IN)
        
        self.timer = self.create_timer(0.2, self.detect)
    
    def detect(self):
        states = {pin: GPIO.input(pin) for pin in PINS} 
        
        self.get_logger().info(f'status: {self.action} \t pinnen: {states}')
        
        if self.action == "Free":
            if 1 in states.values():
                self.action = "Touched"
                self.stop()
                self.timer_10s = self.create_timer(10.0, self.setAvoid)
                
        elif self.action == "Touched":
            if 1 in states.values():
                self.stop()
            else:
                self.action = "Free"
                self.cancelTimer_10s() 
    
        elif self.action == "Avoid":
            self.cancelTimer_10s()
            
            if self.timer_3s is None:
                self.timer_3s = self.create_timer(3.0, self.setStop)
                
            self.drive(states)
    
        elif self.action == "Stop":
            self.cancelTimer_3s()
            self.stop()

            if not 1 in states.values():
                self.action = "Free"

    def cancelTimer_10s(self):
        if self.timer_10s is not None:
            self.timer_10s.cancel()
            self.timer_10s = None
            
    def cancelTimer_3s(self):
        if self.timer_3s is not None:
            self.timer_3s.cancel()
            self.timer_3s = None
    
    def setAvoid(self):
        self.action = "Avoid"
        
    def setStop(self):
        self.action = "Stop"
            
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.bumper_cmd_vel_pub.publish(msg)

    def drive(self, states):
        msg = Twist()

        if states[DIR["V"]] == 0:
            msg.linear.x += speed
        if states[DIR["A"]] == 0:
            msg.linear.x -= speed
        if states[DIR["L"]] == 0:
            msg.linear.y += speed
        if states[DIR["R"]] == 0:
            msg.linear.y -= speed

        self.bumper_cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPIOReaderNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        # rclpy.ok() voorkomt de rcl_shutdown error bij Ctrl+C
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
