#!/usr/bin/env python3
import os
import time
import queue
import threading
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
import socketio

from geometry_msgs.msg import Twist

class QuizSocketBridge(Node):
    """
    QuizSocketBridge: ontvangt socket.io events van de Pi/webapp en vertaalt
    relevante events naar ROS topics / callbacks.

    - Zet 'drive' socket events om naar geometry_msgs/Twist op /gui_cmd_vel.
    - Verwerkt alle socket callbacks thread-safe via een interne queue.
    - Roept state_manager_callback(event, data) aan in de ROS-thread.
    - shutdown() doet socket disconnect + thread join (vernietigt node niet).
    - destroy() vernietigt de ROS node (roep dit aan nadat executor.remove_node(node) is gedaan).
    """

    def __init__(self, state_manager_callback, server_url: str | None = None):
        super().__init__("quiz_socket_bridge")

        self.state_manager_callback = state_manager_callback

        # Manual drive state
        self.last_drive_cmd_time = 0.0
        self.is_moving = False
        self.admin_panel_open = False
        self.manual_drive_db_updated = False

        
        # Publisher voor manuele besturing (GUI)
        self.gui_cmd_vel_pub = self.create_publisher(Twist, '/gui_cmd_vel', 10)
        self.create_subscription(String, "/screen_command", self._on_ros_screen_command, 10)

        # Queue voor thread-safe event verwerking (socket thread -> ROS thread)
        self._event_queue: "queue.Queue[tuple[str, object]]" = queue.Queue()

        # Timer om queue te verwerken en drive timeout te checken (ROS-thread)
        self._timer = self.create_timer(0.1, self._on_timer)

        # Socket.IO client (callbacks draaien in socket thread)
        self.sio = socketio.Client(reconnection=True, reconnection_attempts=5)

        # Run-flag voor nette shutdown
        self._running = True

        # Socket thread handle
        self._sio_thread: threading.Thread | None = None

        # Server URL configuratie
        if server_url is None:
            server_url = os.environ.get("QUIZ_SERVER_URL", "http://10.0.0.11:80")
        self._server_url = server_url

        # Bind socket events: push events naar queue (thread-safe)
        self._bind_socket_events()

        # Start socket.io connect in aparte thread
        self._sio_thread = threading.Thread(target=self._start_socketio, daemon=True)
        self._sio_thread.start()

        self.get_logger().info(f"QuizSocketBridge gestart, probeert te verbinden met {self._server_url}")

    # -----------------------------
    # Socket event binding (socket thread)
    # -----------------------------
    def _bind_socket_events(self):
        # connect / disconnect
        self.sio.on('connect', lambda *args: self._enqueue_event("connect", None))
        self.sio.on('disconnect', lambda *args: self._enqueue_event("disconnect", None))

        # quiz events
        self.sio.on('quiz-finished', lambda data=None: self._enqueue_event("quiz_finished", data))
        self.sio.on('quiz_inactive', lambda data=None: self._enqueue_event("quiz_inactive", data))
        self.sio.on('drive_to_quiz_location', lambda data=None: self._enqueue_event("drive_to_quiz_location", data))

        # screen requests
        self.sio.on('robot-askScreen', lambda data=None: self._enqueue_event("screen_request", data))

        # robot active button
        self.sio.on('robot-askIsActive', lambda data=None: self._enqueue_event("ask_is_active", data))
        self.sio.on('robot-activeButtonToggled', lambda data=None: self._enqueue_event("active_button_toggled", data))

        # admin / safety
        self.sio.on('robot-stop-for-x-time', lambda data=None: self._enqueue_event("admin_login_open", data))
        self.sio.on('login_canceled', lambda data=None: self._enqueue_event("admin_login_closed", data))
        self.sio.on('admin-panel-open', lambda data=None: self._enqueue_event("admin_open", data))
        self.sio.on('admin-panel-closed', lambda data=None: self._enqueue_event("admin_closed", data))

        # battery
        self.sio.on('robot-get-battery-percentage', lambda data=None: self._enqueue_event("battery_request", data))

        # manual drive (belangrijk)
        self.sio.on('drive', lambda data=None: self._enqueue_event("drive", data))

        # time sync
        self.sio.on('time-updated', lambda data=None: self._enqueue_event("time_updated", data))

    # -----------------------------
    # Start socket.io client (in aparte thread)
    # -----------------------------
    def _start_socketio(self):
        try:
            # wait=True zorgt dat connect blokkeert tot connected of timeout
            self.sio.connect(self._server_url, wait=True, transports=['websocket'])
            # geen blocking wait() hier; callbacks blijven in achtergrond
            self.get_logger().info("Socket.IO connectie gestart (background thread).")
        except Exception as e:
            # push connect error naar queue zodat ROS-thread het kan loggen
            self._enqueue_event("connect_error", {"error": str(e)})
            # logger is beschikbaar in deze thread omdat node nog niet destroyed
            try:
                self.get_logger().error(f"Socket.IO connect faalde: {e}")
            except Exception:
                print(f"Socket.IO connect faalde: {e}")

    # -----------------------------
    # Queue helper (thread-safe)
    # -----------------------------
    def _enqueue_event(self, event_name: str, data):
        try:
            self._event_queue.put_nowait((event_name, data))
        except Exception:
            # onwaarschijnlijk met onbegrensde queue, maar log defensief
            try:
                self.get_logger().warn(f"Event queue vol, drop event: {event_name}")
            except Exception:
                print(f"Event queue vol, drop event: {event_name}")

    # -----------------------------
    # Timer callback (ROS-thread): verwerk queue en drive timeout
    # -----------------------------
    def _on_timer(self):
        if not self._running:
            return

        # Verwerk alle events in queue
        while True:
            try:
                event_name, data = self._event_queue.get_nowait()
            except queue.Empty:
                break

            try:
                if event_name == "drive":
                    self._process_drive_event(data)
                elif event_name == "admin_open":
                    # zet lokale flag en forward naar state manager
                    self.admin_open(data)
                    self._safe_state_callback("admin_open", data)
                elif event_name == "admin_closed":
                    self.admin_closed(data)
                    self._safe_state_callback("admin_closed", data)
                elif event_name == "admin_login_open":
                    # zet lokale flag en forward naar state manager
                    self.admin_login_open(data)
                    self._safe_state_callback("admin_login_open", data)
                elif event_name == "admin_login_closed":
                    self.admin_login_closed(data)
                    self._safe_state_callback("admin_closed", data)
                elif event_name == "connect":
                    self.get_logger().info("Socket verbonden (event).")
                    self._safe_state_callback("connect", data)
                elif event_name == "disconnect":
                    self.get_logger().info("Socket verbroken (event).")
                    self._safe_state_callback("disconnect", data)
                elif event_name == "connect_error":
                    self.get_logger().error(f"Socket connect error: {data}")
                elif event_name == "ask_is_active":
                    # We sturen de aanvraag door naar de state manager
                    self._safe_state_callback("ask_is_active", self) # We geven de bridge (self) mee als argument!
                elif event_name == "active_button_toggled":
                    # We geven de data en de bridge (self) mee als argument!
                    self._safe_state_callback("active_button_toggled", (data, self))

                else:
                    # Standaard: forward event naar state manager
                    self._safe_state_callback(event_name, data)
            except Exception as e:
                self.get_logger().error(f"Fout bij verwerken event '{event_name}': {e}")

        # Drive timeout: stop als geen nieuw commando binnen 0.3s
        try:
            if self.is_moving and (time.time() - self.last_drive_cmd_time > 0.3):
                self.get_logger().info("Manual drive stopped (timeout).")
                stop_msg = Twist()  # zero twist
                self.gui_cmd_vel_pub.publish(stop_msg)
                self.is_moving = False
        except Exception as e:
            self.get_logger().error(f"Fout bij drive timeout handling: {e}")


    # -----------------------------
    # UITGAANDE SCHERMWISSELS NAAR PI
    # -----------------------------
    def _on_ros_screen_command(self, msg: String):
        event_name = msg.data
        if not event_name or event_name == "None":
            return

        try:
            if self.sio.connected:
                # Omdat de Pi luistert naar losse events (zoals 'robot-go-charge'),
                # schieten we de tekst van het ROS-bericht direct af als de EVENT-NAAM!
                self.sio.emit(event_name)
                self.get_logger().info(f"🚀 Socket.IO event afgevuurd naar Pi: {event_name}")
            else:
                self.get_logger().warn(f"Kan event '{event_name}' niet sturen: Socket niet verbonden.")
        except Exception as e:
            self.get_logger().error(f"Fout bij verzenden Socket.IO event naar Pi: {e}")



    # -----------------------------
    # Veilige wrapper voor state_manager_callback (ROS-thread)
    # -----------------------------
    def _safe_state_callback(self, event_name: str, data):
        try:
            if callable(self.state_manager_callback):
                self.state_manager_callback(event_name, data)
        except Exception as e:
            self.get_logger().error(f"state_manager_callback fout voor '{event_name}': {e}")

    # ============================================================
    # MANUAL DRIVE (verwerkt in ROS-thread)
    # ============================================================
    def _process_drive_event(self, data):
        """Verwerk 'drive' event en publiceer Twist op /gui_cmd_vel."""
        if not data or not isinstance(data, dict):
            return

        direction = data.get('direction', 'stop')
        raw_speed = data.get('speed', 0.0)

        try:
            speed = float(raw_speed)
        except Exception:
            speed = 0.0

        # clamp speed tussen 0 en 1
        speed = max(0.0, min(1.0, speed))

        self.last_drive_cmd_time = time.time()
        self.is_moving = True

        msg = Twist()

        # Admin panel open → expliciet stoppen
        if self.admin_panel_open:
            try:
                self.gui_cmd_vel_pub.publish(msg)  # zero twist
            except Exception as e:
                self.get_logger().error(f"Publish admin stop failed: {e}")
            return

        lin_scale = 0.15
        ang_scale = 0.15

        if direction == 'forward':
            msg.linear.x = lin_scale * speed
        elif direction == 'backward':
            msg.linear.x = -lin_scale * speed
        elif direction == 'left':
            msg.linear.y = -lin_scale * speed
        elif direction == 'right':
            msg.linear.y = lin_scale * speed
        elif direction == 'cw':
            msg.angular.z = -ang_scale * speed
        elif direction == 'ccw':
            msg.angular.z = ang_scale * speed
        else:
            # stop of onbekend -> zero twist
            msg = Twist()

        try:
            self.gui_cmd_vel_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publish gui_cmd_vel failed: {e}")

    # ============================================================
    # ADMIN PANEL EVENTS (ROS-thread)
    # ============================================================
    def admin_open(self, data=None):
        self.admin_panel_open = True
        self.get_logger().info("Admin panel open → robot moet stoppen.")
        try:
            self.gui_cmd_vel_pub.publish(Twist())
        except Exception as e:
            self.get_logger().error(f"Publish admin stop failed: {e}")

    def admin_closed(self, data=None):
        self.admin_panel_open = False
        self.get_logger().info("Admin panel gesloten → robot mag weer bewegen.")

    def admin_login_open(self, data=None):
        self.admin_login_open = True
        self.get_logger().info("Admin login open → robot moet stoppen.")
        try:
            self.gui_cmd_vel_pub.publish(Twist())
        except Exception as e:
            self.get_logger().error(f"Publish login stop failed: {e}")

    def admin_login_closed(self, data=None):
        self.admin_login_open = False
        self.get_logger().info("Admin login gesloten → robot mag weer bewegen.")

    # ============================================================
    # Shutdown / cleanup (doet NIET destroy_node)
    # ============================================================
    def shutdown(self, join_timeout: float = 1.0):
        """Veilige cleanup: stop verwerking, disconnect socket, join thread.
        Deze methode vernietigt de ROS node niet; roep destroy() daarna als je dat wilt.
        """
        # 1) stop timer/queue verwerking
        self._running = False

        # 2) disconnect socket.io
        try:
            if hasattr(self, 'sio') and self.sio is not None and getattr(self.sio, "connected", False):
                try:
                    self.sio.disconnect()
                    # logger mogelijk nog beschikbaar
                    try:
                        self.get_logger().info("Socket.IO disconnected (shutdown).")
                    except Exception:
                        print("Socket.IO disconnected (shutdown).")
                except Exception as e:
                    try:
                        self.get_logger().warn(f"Socket.IO disconnect faalde: {e}")
                    except Exception:
                        print(f"Socket.IO disconnect faalde: {e}")
        except Exception as e:
            try:
                self.get_logger().warn(f"Fout tijdens socket disconnect check: {e}")
            except Exception:
                print(f"Fout tijdens socket disconnect check: {e}")

        # 3) join socket thread (kort wachten)
        try:
            if self._sio_thread is not None and self._sio_thread.is_alive():
                self._sio_thread.join(timeout=join_timeout)
                if self._sio_thread.is_alive():
                    try:
                        self.get_logger().warn("Socket thread nog actief na join timeout.")
                    except Exception:
                        print("Socket thread nog actief na join timeout.")
        except Exception as e:
            try:
                self.get_logger().warn(f"Fout bij join socket thread: {e}")
            except Exception:
                print(f"Fout bij join socket thread: {e}")

    # ============================================================
    # Destroy node resources explicitly (roep dit aan nadat executor.remove_node(node) is gedaan)
    # ============================================================
    def destroy(self):
        """Vernietig de ROS node resources. Roep dit aan nadat de node uit de executor is verwijderd."""
        try:
            self.destroy_node()
            # logger is niet gegarandeerd na destroy_node, dus geen verdere logging hier
        except Exception:
            # fallback: niets doen
            pass

    # -----------------------------
    # STATUS TERUGSTUREN NAAR PI
    # -----------------------------
    def send_active_status_to_pi(self, is_active: bool):
        try:
            if self.sio.connected:
                # We sturen een puur databericht ZONDER de gevaarlijke toggled-eventnaam
                self.sio.emit('robot-isActive', {'active': is_active})
                self.get_logger().info(f"🟢 Status succesvol verzonden via robot-isActive: {is_active}")
        except Exception as e:
            self.get_logger().error(f"Fout bij verzenden actieve status naar Pi: {e}")

# -----------------------------
# main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)

    # state manager import (zorg dat dit module in PYTHONPATH staat)
    from robot_state_manager import RobotStateManager
    state_manager = RobotStateManager()

    bridge = QuizSocketBridge(state_manager.handle_socket_event)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(state_manager)
    executor.add_node(bridge)

    try:
        executor.spin()
    finally:
        # nette cleanup zonder gebruik van bridge.get_logger() nadat node mogelijk destroyed is
        # 1) verwijder node uit executor zodat executor niet meer verantwoordelijk is
        try:
            executor.remove_node(bridge)
        except Exception:
            # ignore if not registered or already removed
            pass

        # 2) shutdown bridge (disconnect socket + join thread)
        try:
            bridge.shutdown()
        except Exception as e:
            # gebruik print als fallback zodat we niet crashen op logger calls
            print(f"bridge.shutdown() faalde: {e}")

        # 3) expliciet destroy node resources (veilig omdat we de node uit executor verwijderden)
        try:
            bridge.destroy()
        except Exception as e:
            print(f"bridge.destroy() faalde: {e}")

        # 4) stop executor en rclpy
        try:
            executor.shutdown()
        except Exception:
            pass

        rclpy.shutdown()

if __name__ == "__main__":
    main()
