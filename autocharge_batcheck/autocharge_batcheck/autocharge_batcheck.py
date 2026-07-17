#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import json
import time
from collections import deque


from std_msgs.msg import Float32


BATSTATUS_FILE = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt"

VOLTAGE_CONFIG = "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/trees/spanningsniveaus.json"


LOW_COUNT_REQUIRED = 5
OK_COUNT_REQUIRED = 5


LOW_TO_OK_DELAY = 300   # 5 minuten


# sliding window instellingen
WINDOW_SIZE = 20


# gemiddelde spanning publiceren
AVERAGE_PUBLISH_INTERVAL = 180   # 3 minuten



class AutochargeBatcheck(Node):

    def __init__(self):

        super().__init__('autocharge_batcheck')


        self.low_voltage = None
        self.ok_voltage = None


        self.low_counter = 0
        self.ok_counter = 0


        self.last_status = None
        self.low_detected_time = None


        self.voltage_window = deque(maxlen=WINDOW_SIZE)


        self.load_voltage_config()


        self.subscription = self.create_subscription(
            Float32,
            '/PowerVoltage',
            self.voltage_callback,
            10
        )


        self.average_voltage_pub = self.create_publisher(
            Float32,
            '/BatteryAverageVoltage',
            10
        )


        self.average_timer = self.create_timer(
            AVERAGE_PUBLISH_INTERVAL,
            self.publish_average_voltage
        )


        self.get_logger().info(
            "autocharge_batcheck gestart"
        )



    def load_voltage_config(self):

        try:

            with open(VOLTAGE_CONFIG, "r") as f:
                data = json.load(f)


            self.low_voltage = data["battery_low_voltage"]
            self.ok_voltage = data["battery_ok_voltage"]


            self.get_logger().info(
                f"Voltage grenzen geladen LOW={self.low_voltage} OK={self.ok_voltage}"
            )


        except Exception as e:

            self.get_logger().error(
                f"Kan spanningsconfig niet lezen: {e}"
            )

            raise



    def write_status(self, status):

        try:

            with open(BATSTATUS_FILE, "w") as f:
                f.write(status + "\n")


            self.last_status = status


            self.get_logger().warn(
                f"Battery status -> {status}"
            )


        except Exception as e:

            self.get_logger().error(
                f"Kan status file niet schrijven: {e}"
            )



    def get_average_voltage(self):

        if len(self.voltage_window) == 0:
            return None


        return sum(self.voltage_window) / len(self.voltage_window)



    def publish_average_voltage(self):

        avg = self.get_average_voltage()


        if avg is None:
            return


        msg = Float32()
        msg.data = float(avg)

        self.average_voltage_pub.publish(msg)


        self.get_logger().info(
            f"Gemiddelde batterijspanning gepubliceerd: {avg:.2f}V"
        )



    def voltage_callback(self, msg):

        voltage = msg.data


        # waarde toevoegen aan sliding window

        self.voltage_window.append(voltage)


        avg_voltage = self.get_average_voltage()


        if avg_voltage is None:
            return



        self.get_logger().info(
            f"Voltage {voltage:.2f}V  gemiddelde {avg_voltage:.2f}V"
        )



        #
        # BATTERIJ LAAG
        #

        if avg_voltage < self.low_voltage:


            self.low_counter += 1
            self.ok_counter = 0


            self.get_logger().info(
                f"LOW gemiddelde teller {self.low_counter}/{LOW_COUNT_REQUIRED}"
            )


            if self.low_counter >= LOW_COUNT_REQUIRED:


                if self.last_status != "BATTERY-LOW":

                    self.write_status(
                        "BATTERY-LOW"
                    )


                # altijd tijd vernieuwen bij echte LOW detectie

                self.low_detected_time = time.time()



        #
        # BATTERIJ OK
        #

        elif avg_voltage > self.ok_voltage:


            self.ok_counter += 1
            self.low_counter = 0


            self.get_logger().info(
                f"OK gemiddelde teller {self.ok_counter}/{OK_COUNT_REQUIRED}"
            )


            if self.ok_counter >= OK_COUNT_REQUIRED:


                if self.low_detected_time is not None:


                    elapsed = time.time() - self.low_detected_time


                    if elapsed < LOW_TO_OK_DELAY:


                        remaining = LOW_TO_OK_DELAY - elapsed


                        self.get_logger().info(
                            f"Nog {remaining:.0f}s wachten voor BATTERY-OK"
                        )

                        return



                if self.last_status != "BATTERY-OK":

                    self.write_status(
                        "BATTERY-OK"
                    )



        #
        # HYSTERESIS GEBIED
        #

        else:

            self.low_counter = 0
            self.ok_counter = 0



def main(args=None):

    rclpy.init(args=args)


    node = AutochargeBatcheck()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
