import rclpy
from rclpy.node import Node
import Jetson.GPIO as GPIO
from geometry_msgs.msg import Twist

PINS = [7, 15, 31, 32]
DIR = {"V": 15, "A": 32, "R": 7, "L": 31}  # Definieer Voor, Achter, Links, Rechts
speed = 0.1

class GPIOReaderNode(Node):
	def __init__(self):
		super().__init__('gpio_reader_node')
		
		self.bumper_cmd_vel_pub = self.create_publisher(Twist, '/bump_cmd_vel', 1)

		self.drive = False
		self.recovery_timer = None

		GPIO.setmode(GPIO.BOARD)  
		for pin in PINS:
			GPIO.setup(pin, GPIO.IN)
		
		self.timer = self.create_timer(0.2, self.detect)
	
	def detect(self):
		states = {pin: GPIO.input(pin) for pin in PINS}
		
		if 0 in states.values():
			self.emergency(states)
			
			if self.recovery_timer is None and not self.drive:
				self.get_logger().info("Botsing gedetecteerd! Wacht 10 seconden...")
				self.recovery_timer = self.create_timer(10.0, self.recovery)

		elif set(states.values()) == {1}:

			if self.drive or self.recovery_timer is not None:
				self.stop()
				self.drive = False
				
				if self.recovery_timer is not None:
					self.recovery_timer.cancel()
					self.recovery_timer = None 
				
				self.get_logger().info("Bumper weer vrij. Nav2 neemt over.")

	def recovery(self):
		self.drive = True
		self.get_logger().info("10 seconden voorbij, ontwijken gestart.")
		
		if self.recovery_timer is not None:
			self.recovery_timer.cancel()
			self.recovery_timer = None

	def emergency(self, states):
		msg = Twist()
		msg.linear.x = 0.0
		msg.linear.y = 0.0

		if self.drive == True:
			if states[DIR["V"]] == 0:
				msg.linear.x += speed
			if states[DIR["A"]] == 0:
				msg.linear.x -= speed
			if states[DIR["L"]] == 0:
				msg.linear.y += speed
			if states[DIR["R"]] == 0:
				msg.linear.y -= speed

		self.bumper_cmd_vel_pub.publish(msg)
	
	def stop(self):
		msg = Twist()
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
		rclpy.shutdown()

if __name__ == '__main__':
	main()

