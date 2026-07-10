#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import json
import time
import os


from std_msgs.msg import Float32


BATSTATUS_FILE = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt"

VOLTAGE_CONFIG = "/home/wheeltec/wheeltec_ros2/src/mecabot_bt/trees/spanningsniveaus.json"


LOW_COUNT_REQUIRED = 5
OK_COUNT_REQUIRED = 5

LOW_TO_OK_DELAY = 300   # 5 minuten


class AutochargeBatcheck(Node):

    def __init__(self):

        super().__init__('autocharge_batcheck')


        self.low_voltage = None
        self.ok_voltage = None


        self.low_counter = 0
        self.ok_counter = 0


        self.last_status = None
        self.low_detected_time = None


        self.load_voltage_config()


        self.subscription = self.create_subscription(
            Float32,
            '/PowerVoltage',
            self.voltage_callback,
            10
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
                f"Voltage grenzen geladen: LOW={self.low_voltage} OK={self.ok_voltage}"
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

            self.get_logger().info(
                f"Battery status veranderd naar {status}"
            )


        except Exception as e:

            self.get_logger().error(
                f"Kan status file niet schrijven: {e}"
            )


    def voltage_callback(self, msg):

        voltage = msg.data


        self.get_logger().info(
            f"Voltage gemeten: {voltage:.2f}V"
        )


        # batterij laag detectie

        if voltage < self.low_voltage:

            self.low_counter += 1
            self.ok_counter = 0


            self.get_logger().info(
                f"Lage spanning teller: {self.low_counter}/{LOW_COUNT_REQUIRED}"
            )


            if self.low_counter >= LOW_COUNT_REQUIRED:

                if self.last_status != "BATTERY-LOW":

                    self.write_status("BATTERY-LOW")
                    self.low_detected_time = time.time()



        # batterij goed detectie

        elif voltage > self.ok_voltage:


            self.ok_counter += 1
            self.low_counter = 0


            self.get_logger().info(
                f"Goede spanning teller: {self.ok_counter}/{OK_COUNT_REQUIRED}"
            )


            if self.ok_counter >= OK_COUNT_REQUIRED:


                # eerst controleren of 5 minuten voorbij zijn

                if self.low_detected_time is not None:

                    elapsed = time.time() - self.low_detected_time


                    if elapsed < LOW_TO_OK_DELAY:

                        remaining = LOW_TO_OK_DELAY - elapsed

                        self.get_logger().info(
                            f"Wacht nog {remaining:.0f}s voor BATTERY-OK"
                        )

                        return


                if self.last_status != "BATTERY-OK":

                    self.write_status("BATTERY-OK")



        else:

            # spanning tussen LOW en OK
            # reset tellers

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
