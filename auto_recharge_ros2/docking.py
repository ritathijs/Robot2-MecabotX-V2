#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

class TagDocker(Node):
    def __init__(self):
        super().__init__('tag_docker')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # NIEUWE AFSTAND: 50 centimeter (0.5 meter)
        self.target_distance = 0.50  
        self.get_logger().info("Visual Servoing gestart! Wachten op tag36h11:1 (Doel: 50 cm)...")

    def control_loop(self):
        try:
            # We luisteren direct naar de camera om afhankelijkheid van de map te voorkomen
            trans = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'tag36h11:1', rclpy.time.Time())
        except Exception:
            # Veiligheid: stop de robot als de tag uit beeld verdwijnt
            self.cmd_pub.publish(Twist())
            return

        # 1. POSITIES (Optisch frame: Z = Diepte, X = Zijwaarts)
        afstand_naar_voren = trans.transform.translation.z
        afstand_naar_rechts = trans.transform.translation.x
        
        # 2. ROTATIE (De hoek van de muur)
        q = trans.transform.rotation
        Nx = 2.0 * (q.x * q.z + q.w * q.y)
        Nz = 1.0 - 2.0 * (q.x**2 + q.y**2)
        skew_angle = math.atan2(Nx, -Nz) 
        
        # Foutmarges berekenen
        error_forward = afstand_naar_voren - self.target_distance
        error_lateral = afstand_naar_rechts
        error_yaw = skew_angle
        
        msg = Twist()
        
        # Stop-conditie: Afstand < 2cm, Midden < 2cm, én Hoek < 3 graden
        if abs(error_forward) < 0.02 and abs(error_lateral) < 0.02 and abs(error_yaw) < 0.05:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("DOEL BEREIKT! Robot is perfect gepositioneerd op 50 cm.")
        else:
            # --- 1. Vooruit/Achteruit ---
            snelheid_x = error_forward * 0.5
            msg.linear.x = min(max(snelheid_x, -0.15), 0.15)
            
            # --- 2. Zijdelings schuiven (Strafing) ---
            # Minteken behouden: Dit zorgde voor de perfecte cirkelbaan om de tag!
            snelheid_y = -error_lateral * 0.6
            msg.linear.y = min(max(snelheid_y, -0.15), 0.15)
            
            # --- 3. Roteren (Recht trekken) ---
            # Minteken weggehaald: Zodat hij nu wijzerzin draait zoals vereist!
            snelheid_draaien = error_yaw * 0.8
            msg.angular.z = min(max(snelheid_draaien, -0.3), 0.3)
            
            hoek_graden = math.degrees(skew_angle)
            self.get_logger().info(f"Afstand: {afstand_naar_voren:.2f}m | Zijwaarts: {afstand_naar_rechts:.2f}m | Scheefheid: {hoek_graden:.1f}°")

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TagDocker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nNoodstop geactiveerd! Robot stopt.")
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
