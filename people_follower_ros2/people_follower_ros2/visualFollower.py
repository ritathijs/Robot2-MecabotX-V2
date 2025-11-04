#!/usr/bin/env python3

# DIT WERKTTTTTT

import rclpy
import _thread
import threading
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from turn_on_wheeltec_robot.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


angle=[0.0]*3
distan=[0.0]*3

class VisualFollower(Node):
    def __init__(self):
        super().__init__('visualfollower')
        qos = QoSProfile(depth=10)

        # ---------- Params for safe stopping ----------
        self.declare_parameter('stop_on_shutdown', True)
        self.declare_parameter('stop_repeats', 5)        # send stop N times
        self.declare_parameter('stop_interval', 0.05)    # seconds between stops
        self.declare_parameter('position_timeout_s', 1.0)# watchdog timeout on Position stream

        self.stop_on_shutdown = bool(self.get_parameter('stop_on_shutdown').value)
        self.stop_repeats     = int(self.get_parameter('stop_repeats').value)
        self.stop_interval    = float(self.get_parameter('stop_interval').value)
        self.position_timeout = float(self.get_parameter('position_timeout_s').value)

        # as soon as we stop receiving Joy messages from the ps3 controller we stop all movement:
        self.controllerLossTimer = threading.Timer(1, self.controllerLoss) #if we lose connection
        self.controllerLossTimer.start()

        self.switchMode= True  # if False: O must be held down to move
        self.max_speed = 0.3
        self.controllButtonIndex = -4

        self.buttonCallbackBusy=False
        self.active=False
        self.i=0

        self.cmdVelPublisher = self.create_publisher(Twist, '/cmd_vel', qos)

        # tracker subscriptions
        self.positionSubscriber = self.create_subscription(
            PositionMsg, '/object_tracker/current_position', self.positionUpdateCallback, qos)
        self.trackerInfoSubscriber = self.create_subscription(
            StringMsg, '/object_tracker/info', self.trackerInfoCallback, qos)

        # PID parameters first is angular, dist
        targetDist = 600  # EXPECTS millimeters
        self.PID_controller = simplePID([0, targetDist], [1.2 ,0.2 ], [0 ,0.00], [0.005 ,0.00])

        # --- Position watchdog (stops if no new Position in time) ---
        self._last_position_time = None
        self._watchdog_timer = self.create_timer(0.1, self._position_watchdog_cb)

    def _position_watchdog_cb(self):
        if self._last_position_time is None:
            return
        now = self.get_clock().now()
        dt = (now - self._last_position_time).nanoseconds * 1e-9
        if dt > self.position_timeout:
            # no fresh position => stop
            self.stopMoving()
            # don’t spam logs; print only once per lapse
            self._last_position_time = None
            self.get_logger().info(f'Position timeout ({dt:.2f}s). Stopping.')

    def trackerInfoCallback(self, info):
        # optional info from tracker
        self.get_logger().warn(info.data)

    def positionUpdateCallback(self, position: PositionMsg):
        # record time for watchdog
        self._last_position_time = self.get_clock().now()

        # If you want joystick “active” gating, uncomment:
        # if not self.active:
        #     return

        angleX  = position.angle_x       # radians
        distance = position.distance     # controller expects mm (keep it this way)

        # PID update: [ang(rad), dist(mm)]
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angleX, distance])

        # Clip speeds
        angularSpeed = float(np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed))
        linearSpeed  = float(np.clip(-uncliped_lin_speed,  -self.max_speed, self.max_speed))

        velocity = Twist()
        velocity.linear.x  = linearSpeed
        velocity.linear.y  = 0.0
        velocity.linear.z  = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = angularSpeed

        # guard: if distance invalid or too far, stop
        if (distance > 5000) or (distance == 0):
            self.stopMoving()
            self.get_logger().debug('Out of tracking range; stopping.')
        else:
            self.cmdVelPublisher.publish(velocity)

    def buttonCallback(self, joy_data: Joy):
        # keep-alive for controller loss
        self.controllerLossTimer.cancel()
        self.controllerLossTimer = threading.Timer(0.5, self.controllerLoss)
        self.controllerLossTimer.start()

        # drop rapid repeats using a worker thread
        if self.buttonCallbackBusy:
            return
        else:
            _thread.start_new_thread(self.threadedButtonCallback, (joy_data, ))

    def threadedButtonCallback(self, joy_data: Joy):
        self.buttonCallbackBusy = True

        if (joy_data.buttons[self.controllButtonIndex] == self.switchMode) and self.active:
            self.get_logger().info('Stopping (button).')
            self.stopMoving()
            self.active = False
            time.sleep(0.5)
        elif (joy_data.buttons[self.controllButtonIndex] == True) and (not self.active):
            self.get_logger().info('Activating (button).')
            self.active = True
            time.sleep(0.5)

        self.buttonCallbackBusy = False

    def stopMoving(self):
        """Publish one zero Twist."""
        velocity = Twist()
        # all zeros
        self.cmdVelPublisher.publish(velocity)

    def stopBurst(self):
        """Publish several zero Twists to ensure the base halts."""
        try:
            zero = Twist()
            for _ in range(self.stop_repeats):
                self.cmdVelPublisher.publish(zero)
                time.sleep(self.stop_interval)
            self.get_logger().info('Sent stop burst to /cmd_vel.')
        except Exception as e:
            self.get_logger().warn(f'Failed to publish stop burst: {e}')

    def controllerLoss(self):
        # we lost controller connection => stop and become inactive
        self.stopMoving()
        self.active = False
        self.get_logger().info('lost controller connection')


class simplePID:
    '''very simple discrete PID controller'''
    def __init__(self, target, P, I, D):
        # check shapes
        if (not (np.size(P) == np.size(I) == np.size(D)) or
            ((np.size(target) == 1) and np.size(P) != 1) or
            (np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')

        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.setPoint = np.array(target)

        self.last_error = 0
        self.integrator = 0
        self.integrator_max = float('inf')
        self.timeOfLastCall = None

    def update(self, current_value):
        current_value = np.array(current_value)
        if np.size(current_value) != np.size(self.setPoint):
            raise TypeError('current_value and target do not have the same shape')

        if self.timeOfLastCall is None:
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))

        error = self.setPoint - current_value
        # when bias is little, stop moving. error[0]=angle(rad), error[1]=distance(mm)
        if -0.1 < error[0] < 0.1:
            error[0] = 0.0
        if -150 < error[1] < 150:
            error[1] = 0.0

        # when target is small, amplify (approach strongly)
        if (error[1] > 0) and (self.setPoint[1] < 1200):
            error[1] = error[1] * (1200 / self.setPoint[1]) * 0.5
            error[0] = error[0] * 0.8

        P = error

        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)

        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator

        D = (error - self.last_error) / max(deltaT, 1e-6)

        self.last_error = error
        self.timeOfLastCall = currentTime

        return self.Kp * P + self.Ki * I + self.Kd * D


def main(args=None):
    print('visualFollower')
    rclpy.init(args=args)
    visualFollower = VisualFollower()
    print('visualFollower init done')
    try:
        rclpy.spin(visualFollower)
    except KeyboardInterrupt:
        pass
    finally:
        # stop timers first
        try:
            visualFollower.controllerLossTimer.cancel()
        except Exception:
            pass
        # send a burst of zeros so the base actually halts
        if visualFollower.stop_on_shutdown:
            visualFollower.stopBurst()
        # destroy node and shutdown ROS
        visualFollower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

