#!/usr/bin/env python3 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python的安装路径，再调用对应路径下的解析器完成操作
#2:Python.源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

#引用ros库
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  

from rclpy.qos import QoSProfile, ReliabilityPolicy

# 用到的变量定义
from std_msgs.msg import Bool 
from std_msgs.msg import Int8 
from std_msgs.msg import UInt8
from std_msgs.msg import Float32
from turtlesim.srv import Spawn

# 用于记录充电桩位置、发布导航点
from geometry_msgs.msg import PoseStamped

# rviz可视化相关
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# cmd_vel话题数据
from geometry_msgs.msg import Twist

# 里程计话题相关
from nav_msgs.msg import Odometry


from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 获取导航结果
# from move_base_msgs.msg import MoveBaseActionResult # ROS1

# 键盘控制相关
import sys, select, termios, tty

# 延迟相关
import time

# 读写充电桩位置文件
import json
import yaml

import math
import os

#存放充电桩位置的文件位置
json_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/Charger_Position.json'
yaml_file='/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/robot_info.yaml'
batstatus_file = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/batstatus.txt"
laadstatus_file = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/laadstatus.txt"

# NIEUW: file waarin we bijhouden of de robot in een laadcyclus zit (LAADCYCLUS) of los staat (LOS)
laadcyclus_file = "/home/wheeltec/wheeltec_ros2/src/auto_recharge_ros2/laadcyclus_status.txt"


#print_and_fixRetract相关，用于打印带颜色的信息
RESET = '\033[0m'
RED   = '\033[1;31m'
GREEN = '\033[1;32m'
YELLOW= '\033[1;33m'
BLUE  = '\033[1;34m'
PURPLE= '\033[1;35m'
CYAN  = '\033[1;36m'

#圆周率
PI=3.1415926535897

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
    
settings = None
if os.name != 'nt' and sys.stdin.isatty():
    settings = list(termios.tcgetattr(sys.stdin))

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    if sys.stdin.isatty():
        tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    if sys.stdin.isatty():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_and_fixRetract(str):
    global settings
    '''键盘控制会导致回调函数内使用print()出现自动缩进的问题，此函数可以解决该现象'''
    if sys.stdin.isatty():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(str)

class AutoRecharger(Node):
    def __init__(self):
        
        #创建节点
        super().__init__("auto_recharger")

        print_and_fixRetract('Automatic charging node start!')


        self.must_reset = False  # VLAG VOOR HET HERSTARTEN VAN DE GEHELE CODE NA STOP BERICHT

 #Statusvariabelen van de robot: type, batterijcapaciteit, batterijspanning, laadstatus, laadstroom, status van het infraroodsignaal, registratie van de houding van de robot        
        self.robot = {
        'Type':'Plus', 
        'BatteryCapacity':5000, 
        'Voltage':25, 
        'Charging':0, 
        'Charging_current':0, 
        'RED':0, 
        'Rotation_Z':0,
        'car_mode':'mini_mec'
        }

        self.charge_session_id = 0 # ID VOOR BERICHTEN VAN EN NAAR BT

        # HET VERBINDINGSPROFIEL VOOR BERICHTEN VAN AUTOCHARGE NAAR BT : ZEKERHEID VAN AANKOMEN
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.Event_pub = self.create_publisher(
            String,
            '/auto_recharge_event',
            qos
        )

        # Verbinding met positie-reset-code
        self.Reset_Position_pub = self.create_publisher(
            String,
            '/resetPositionChargeStation',
            qos
        )

        
        self.charge_xstop_sub = self.create_subscription(
                    String,
                    '/charge_XSTOP',
                    self.charge_xstop_callback,
                    1
        )

        # Schermen aanvragen
        self.rpi_topic_pub = self.create_publisher(
            String,
            "rpitopic",
            10
        )

        # Verbinding met BT
        self.force_charge_sub = self.create_subscription(
            String,
            '/force_charge',
            self.force_charge_callback,
            10
        )

        docking_qos = QoSProfile(depth=1)
        docking_qos.reliability = ReliabilityPolicy.RELIABLE
        docking_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # self.Docking_Status_pub = self.create_publisher(
        #     String,
        #     '/infrared_docking_status',
        #     docking_qos
        # )
        


        self.battery_full_sent = False
        
        self.last_batstatus = None

        self.force_charge_active = False  # NA START MOET EEN STOP KOMEN, EEN NIEUWE START WORDT GENEGEERD (VLAG HOOG)

        #Voor het vastleggen van de positie van de robot op de Z-as aan het einde van de navigatie
        
        self.nav_end_z=0
        self.start_turn = 0
        self.find_redsignal = 0

        #Aantal infraroodsignalen
        self.red_count=0

        # 2de stop moet genegeerd worden
        self.stop_processed = False
        
        #Vlag voor automatische terugkeermodus van de robot: 0: terugkeer uitgeschakeld, 1: navigatie-terugkeer, 2: terugkeer via besturing van de uitrusting
        self.chargeflag=0

        #Variabele voor het vastleggen van tijdstempels door robots
        self.last_time= self.get_clock().now()
        self.lost_red_flag=self.get_clock().now()


        #Aantal gevallen waarin de batterij van de robot te laag is (<12,5 of <25)
        self.power_lost_count=0

        # Robot lage batterij detectie éénmalige vlag
        self.lost_power_once = 1

        # Robot laad voltooid vlag
        self.charge_complete = 0

        # Robot laad voltooid vlag
        self.last_charge_complete = 0

        # Laatste laadpaal locatie data
        self.json_data = 0

        # default waarde voor hoeveel afstand voor pijl rviz er moet worden gereden (waarde uit yaml zal deze overschrijven)
        self.diff_point = 1.2

        # idem hierboven maar dan hoek
        self.diff_angle = -15

 
        #De locatiegegevens van laadpalen uit een JSON-bestand ophalen
        with open(json_file,'r')as fp:
            self.json_data = json.load(fp)

        self.robot_security_off_pub = self.create_publisher(Int8,'/chassis_security',   10) 

        #创建充电桩位置标记话题发布者
        self.Charger_marker_pub   = self.create_publisher(MarkerArray,'/goal_marker',   10) 

        #创建自动回充任务是否开启标志位话题发布者
        self.Recharger_Flag_pub = self.create_publisher(Int8,"robot_recharge_flag",  5)

        #速度话题用于不开启导航时，向底盘发送开启自动回充任务命令
        self.Cmd_vel_pub = self.create_publisher(Twist,"/charge_cmd_vel",  5)


        # Voorzien voor het publiceren van batterijpercentage richting de communicatiecode BT <=> Quiz
        self.Battery_pub = self.create_publisher(Int8, '/battery_percentage', 10)
        self.last_battery_percent = -1

        #创建机器人电量话题订阅者
        self.Voltage_sub = self.create_subscription(Float32, "PowerVoltage", self.Voltage_callback,10)

        #topic waar spanningsniveau opkomt
        #self.Charging_Flag_sub = self.create_subscription(Bool, "robot_charging_flag",self.Charging_Flag_callback,10)

        #topic met laadstroom
        self.Charging_Current_sub = self.create_subscription(Float32,"robot_charging_current",  self.Charging_Current_callback,10)

        #Abonnees op het onderwerp 'Robot heeft infraroodsignaal gedetecteerd'
        self.RED_Flag_sub = self.create_subscription(UInt8,"robot_red_flag",  self.RED_Flag_callback,10)

                #Abonnees op het onderwerp 'Locaties van laadpalen bijwerken'
        self.Charger_Position_Update_sub = self.create_subscription( PoseStamped,"/charger_position_update", self.Position_Update_callback,10)

        #odomoetrie info
        self.Odom_sub = self.create_subscription(Odometry, '/odom', self.Odom_callback,10)

        

        # 创建服务调用者
        self.set_charge = self.create_client(Spawn,'/set_charge')

        self.server_set_state = None
        self.wait_server_done = None
        #按键控制说明
        self.tips = """
使用下面按键使用自动回充功能.       Press below Key to AutoRecharger.
Q/q:开启自动回充.                   Q/q:Start looking for charger.
E/e:停止自动回充.                   E/e:Stop find charger.
Ctrl+C/c:关闭自动回充功能并退出.    Ctrl+C/c:Quit the program.
可使用话题"charger_position_update"更新充电桩的位置.
        """

    # Callback voor verbinding met STM
    # slaat resultaat van service op en markeren dat response binnen is
        
    def wait_server_callback(self,res):
        self.wait_server_done = 1  # serivce response is ontvangen
        response = res.result()   # ophalen van het response zelf
        self.server_set_state = response.name   # opslaan van de status (bv true of false)

    def publish_rpi_event(self, text):
        msg = String()
        msg.data = text
        self.rpi_topic_pub.publish(msg)
        print_and_fixRetract(GREEN + f"RPITOPIC -> {text}" + RESET)



    # def publish_docking_status(self, status):

    #     if not self.wait_for_docking_status_subscriber(timeout=5.0):
    #         print_and_fixRetract(
    #             RED + "Docking status niet verstuurd: geen ontvanger." + RESET
    #         )
    #         return


    #     msg = String()
    #     msg.data = status

    #     self.Docking_Status_pub.publish(msg)

    #     print_and_fixRetract(
    #         CYAN + f"Docking status -> {status}" + RESET
    #     )

    def write_battery_status(self, status):
        pass
            # """
            # Schrijft BATTERY-LOW of BATTERY-OK naar batstatus.txt.
            # Schrijft alleen wanneer de status veranderd is.
            # """

            # if status == self.last_batstatus:
            #     return

            # try:
            #     with open(batstatus_file, "w") as f:
            #         f.write(status)

            #     self.last_batstatus = status
            #     print_and_fixRetract(GREEN + f"Battery status opgeslagen: {status}" + RESET)

            # except Exception as e:
            #     print_and_fixRetract(RED + f"Fout bij schrijven batstatus.txt: {e}" + RESET)

    
    # Stuurt msg naar auto_recharge_event (BT)
    # Hij plaatst er eerst de juiste integer voor, zodat BT de juistheid kan herkenenn
    def publish_event(self, msg):
        """Publish an auto recharge event to /auto_recharge_event"""

        if self.charge_session_id is not None:
            msg = f"{self.charge_session_id}{msg}"

        event_msg = String()
        event_msg.data = msg
        self.Event_pub.publish(event_msg)

        print_and_fixRetract(msg)

    # NON BLOCKING stop commando naar chasis (stm) sturen
    # Hij gebruikt dus een assynchrone service call om de autocharge UIT te schakelen
    def set_charge_mode_stop(self, value):
        """Non-blocking set charge mode voor Stop_Charge"""
        try:

            # Check of service beschikbaar is (max 2 seconden wachten)
            if not self.set_charge.wait_for_service(timeout_sec=2.0):
                print_and_fixRetract(RED+'底盘服务未就绪，无法关闭自动回充'+RESET)
                return  # indien niet beschikbaar returnen (stop de functie)
        except Exception as e:  # fout bij wachten op service? => print dan ook de fout error message e
            print_and_fixRetract(RED+f'等待服务超时: {e}'+RESET)
            return

        req = Spawn.Request()  # maak service request object aan
        req.x = float(value)
        # async call, niet wachten
        self.set_charge.call_async(req).add_done_callback(self.wait_server_callback)
        print_and_fixRetract(f'正在关闭自动回充功能, 请求已发送...')

    def write_laadstatus(self, tekst):
        try:
            with open(laadstatus_file, "w") as f:
                f.write(tekst)
            print_and_fixRetract(GREEN + f"Laadstatus geschreven: {tekst}" + RESET)
        except Exception as e:
            print_and_fixRetract(RED + f"Fout schrijven laadstatus: {e}" + RESET)

    # NIEUW: schrijft LAADCYCLUS (start van een laadcyclus / robot staat tegen het station)
    # of LOS (stopbericht verwerkt, robot is losgekoppeld) naar laadcyclus_file
    def write_laadcyclus(self, tekst):
        try:
            with open(laadcyclus_file, "w") as f:
                f.write(tekst)
            print_and_fixRetract(GREEN + f"Laadcyclus status geschreven: {tekst}" + RESET)
        except Exception as e:
            print_and_fixRetract(RED + f"Fout schrijven laadcyclus status: {e}" + RESET)


    # Aanzetten van laadmodus via service call. Blijft opnieuw proberen tot response
    def set_charge_mode(self,value,max_callcount=10):
        # 注:不可在回调函数调用服务,否则卡死
        # BELANGRIJK : service calls mogen niet in callbacks (kan deadlock veroorzaken)
        try: 
            if not self.set_charge.wait_for_service(2):    # check of service actief  is?
                raise TimeoutError("Service call time out")
    
        except TimeoutError as e:
            print_and_fixRetract(RED+'Het instellen van de automatische herlaadstatus is mislukt. Wachttijd verstreken.'+RESET)
            return

        state = None  # resultaat van serice (later true of false)
        call_time = 0 # aantal pogingen ondernomen


        if round(value)==1 or round(value)==2:
            print("正在开启自动回充功能,等待响应,请确保底盘节点已开启...")
            print("De functie voor automatisch bijvullen wordt geactiveerd. Wacht op een reactie. Zorg ervoor dat de chassis-node is ingeschakeld...")
        else:
            print("正在关闭自动回充功能,等待响应,请确保底盘节点已开启...")
        while True:  # BLIJF PROBEREN TOT SUCCES!
            try:
                req = Spawn.Request()
                req.x = float(value)
                self.set_charge.call_async(req).add_done_callback(self.wait_server_callback)  # assynchrone call + callback voor response te ontvangen

                # 死循环等待响应结果
                while True:
                    rclpy.spin_once(self)  #wacht op response via ros spin loop (telkens callback afgaan)
                    if self.wait_server_done==1:
                        self.wait_server_done = 0
                        break
                # 输出结果
                state = self.server_set_state  #resultaat lezen en opslaan
                
            except Exception  as e:
                print(e)
                state =  "false"


            # afhankelijk van resultaat afhandeling
            if state=="true":
                self.server_set_state = None
                print("回充状态设置成功.")
                print("De instelling voor het opladen is geslaagd")
                # 调用成功,跳出循环
                break
            else:
                # 记录失败的次数
                call_time = call_time + 1
                if call_time>max_callcount:
                    print_and_fixRetract(RED+'尝试与底盘通信多次失败,无法开启自动回充功能,请检查底层设备是否正确.'+RESET)
                    self.server_set_state = None
                    break
            
            time.sleep(0.5)


    # afhandeling van XSTOP bericht
    def charge_xstop_callback(self, msg):
        command = msg.data.strip().upper()

        if command in ["XSTOP", "XSTOPS"]:
            print_and_fixRetract(
                YELLOW + f"Ontvangen op /charge_XSTOP: {command} (sessie {self.charge_session_id})." + RESET
            )

            # XSTOPS = altijd forceren
            if command == "XSTOPS" or self.robot['Charging'] == 1:
                self.force_charge_active = False

                reset_msg = String()
                reset_msg.data = "RESET"
                self.Reset_Position_pub.publish(reset_msg)

                print_and_fixRetract(
                    GREEN + "Volledige stop + vooruit rijden (XSTOP/XSTOPS)." + RESET
                )

                self.Stop_Charge(drive_forward=True)

            else:
                print_and_fixRetract(
                    YELLOW+ "Robot laadt niet, negeer XSTOP" + RESET
            )

                    
    def force_charge_callback(self, msg):
        command = msg.data.strip().upper()

        # Controleer of het eerste karakter een cijfer is (sessie-ID)
        if len(command) > 1 and command[0].isdigit():
            session_id = int(command[0])
            command_body = command[1:]

            # Eerste START: als nog geen force_charge actief, accepteer het sessienummer (ook na een XSTOP reset)
            if command_body == "START" and not self.force_charge_active:
                if self.robot['Charging'] == 1:
                    print_and_fixRetract(YELLOW + f"START ({command}) genegeerd: Robot is fysiek al aan het laden." + RESET)
                    return
                print_and_fixRetract(YELLOW + f"Eerste START ontvangen, sessie {session_id} wordt actief." + RESET)
                self.charge_session_id = session_id
                self.force_charge_active = True
                self.stop_processed = False
                self.publish_event("DRIVE-TO-DOCK-SUCCESS")
                # Goed startbericht ontvangen -> begin van een laadcyclus
                self.write_laadcyclus("LAADCYCLUS")
                self.start_forced_charging()
                return

            # Volgende berichten controleren tegen de actieve sessie
            if session_id != self.charge_session_id:
                print_and_fixRetract(YELLOW + f"Ontvangen session {session_id}, verwacht {self.charge_session_id}. Ignored." + RESET)
                return

            command = command_body  # strip sessie-nummer voor verdere checks

        # START
        if command == "START":
            if self.force_charge_active:
                print_and_fixRetract(YELLOW + f"START genegeerd: laden al actief (sessie {self.charge_session_id})." + RESET)
                return
            print_and_fixRetract(YELLOW + f"Force charge START ontvangen (sessie {self.charge_session_id})." + RESET)
            self.force_charge_active = True
            self.stop_processed = False
            # Goed startbericht ontvangen -> begin van een laadcyclus
            self.write_laadcyclus("LAADCYCLUS")
            self.start_forced_charging()

        # STOP
        elif command == "STOP":
            # Hier wordt vastgelegd dat de robot NIET (meer) tegen het station staat.
            # Dit is de bestaande write en blijft ongewijzigd; hier wordt NIETS extra's
            # naar onze laadcyclus_file geschreven.
            self.write_laadstatus("LOS")

            if self.stop_processed:
                print_and_fixRetract(
                    YELLOW + f"STOP genegeerd: stop reeds uitgevoerd (sessie {self.charge_session_id})." + RESET
                )
                return

            print_and_fixRetract(
                YELLOW + f"Force charge STOP ontvangen (sessie {self.charge_session_id})." + RESET
            )

            self.stop_processed = True
            self.force_charge_active = False

            self.charge_session_id += 1
            if self.charge_session_id > 9:
                self.charge_session_id = 0

            self.Stop_Charge(drive_forward=True)

            # Stopbericht is nu volledig afgehandeld EN de robot is al naar voor gereden
            # -> pas nu LOS wegschrijven naar onze laadcyclus_file
            self.write_laadcyclus("LOS")


    def Pub_Charger_marker(self, p_x, p_y, o_z, o_w):
        '''发布目标点可视化话题'''
        
        # aanmaken van rviz marker voor laadstation


        tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
        tmp_angle = math.radians(-self.diff_angle)
        new_yaw = (tmp_yaw+tmp_angle)/2
        o_z = math.sin(new_yaw)
        o_w = math.cos(new_yaw)

        tmp_yaw = math.atan2(2*(o_w*o_z),1-2*(o_z**2))
        diff_x = math.cos(tmp_yaw)
        diff_y = math.sin(tmp_yaw)
        p_x = p_x - diff_x*self.diff_point
        p_y = p_y - diff_y*self.diff_point

        markerArray = MarkerArray()

        marker_shape  = Marker() #创建marker对象
        marker_shape.id = 0 #必须赋值id
        marker_shape.header.frame_id = 'map' #以哪一个TF坐标为原点
        marker_shape.type = Marker.ARROW #TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker_shape.action = Marker.ADD #添加marker
        marker_shape.scale.x = 0.5 #marker大小
        marker_shape.scale.y = 0.05 #marker大小
        marker_shape.scale.z = 0.05 #marker大小，对于字符只有z起作用
        marker_shape.pose.position.x = p_x#字符位置
        marker_shape.pose.position.y = p_y #字符位置
        marker_shape.pose.position.z = 0.1 #msg.position.z #字符位置
        marker_shape.pose.orientation.z = o_z #字符位置
        marker_shape.pose.orientation.w = o_w #字符位置
        marker_shape.color.r = 1.0 #字符颜色R(红色)通道
        marker_shape.color.g = 0.0 #字符颜色G(绿色)通道
        marker_shape.color.b = 0.0 #字符颜色B(蓝色)通道
        marker_shape.color.a = 1.0 #字符透明度
        markerArray.markers.append(marker_shape) #添加元素进数组
        
        marker_string = Marker() #创建marker对象
        marker_string.id = 1 #必须赋值id
        marker_string.header.frame_id = 'map' #以哪一个TF坐标为原点
        marker_string.type = Marker.TEXT_VIEW_FACING #一直面向屏幕的字符格式
        marker_string.action = Marker.ADD #添加marker
        marker_string.scale.x = 0.5 #marker大小
        marker_string.scale.y = 0.5 #marker大小
        marker_string.scale.z = 0.5 #marker大小，对于字符只有z起作用
        marker_string.color.a = 1.0 #字符透明度
        marker_string.color.r = 1.0 #字符颜色R(红色)通道
        marker_string.color.g = 0.0 #字符颜色G(绿色)通道
        marker_string.color.b = 0.0 #字符颜色B(蓝色)通道
        marker_string.pose.position.x = p_x #字符位置
        marker_string.pose.position.y = p_y #字符位置
        marker_string.pose.position.z = 0.1 #msg.position.z #字符位置
        marker_string.pose.orientation.z = o_z #字符位置
        marker_string.pose.orientation.w = o_w #字符位置
        marker_string.text = 'Charger' #字符内容
        markerArray.markers.append(marker_string) #添加元素进数组
        self.Charger_marker_pub.publish(markerArray) #发布markerArray，rviz订阅并进行可视化

    def Pub_Recharger_Flag(self,set_velflag=0):
        # oproepen van de juiste functie om status naar stm te sturen

        print_and_fixRetract(GREEN+"PUB RECHARGER BEREIKT"+RESET)

        # 先开回充，再开导航的情况
        if set_velflag==1:
            topic=Int8()
            topic.data=self.chargeflag
            for i in range(10):
                self.Recharger_Flag_pub.publish(topic)
        
        print_and_fixRetract(GREEN+"PUB RECHARGER NET VOOR BLOCKER"+RESET)

        # INDIEN WE WILLEN STOPPEN : doe non blocking (en laat code herstarten)
        # INDIEN STARTEN MET LADEN : blocking variant werkt wel gewoon
        if self.chargeflag == 0:
            self.set_charge_mode_stop(self.chargeflag)
        else:
            self.set_charge_mode(self.chargeflag)




        print_and_fixRetract(GREEN+"PUB RECHARGER NA BLOCKER"+RESET)

    def Voltage_callback(self, topic):
        '''Update batterij spanning'''

        self.robot['Voltage']=topic.data

    def Charging_Current_callback(self, topic):
        """Update laadstroom en bepaal laadstatus op basis van stroom."""

        previous_charging = self.robot['Charging']

        # laadstroom opslaan
        self.robot['Charging_current'] = topic.data

        # bepaal laadstatus
        current_charging = 1 if topic.data > 0.1 else 0

        # overgang: niet laden -> laden
        if previous_charging == 0 and current_charging == 1:

            # self.publish_docking_status("DOCKING_DISABLED")
            self.publish_event("ROBOT-CHARGING")
            self.publish_rpi_event("RobotCharging")
            self.write_laadstatus("LADEN")

            # Hier wordt vastgelegd dat de robot tegen het station staat (er loopt
            # effectief laadstroom) -> ook LAADCYCLUS wegschrijven naar onze file
            self.write_laadcyclus("LAADCYCLUS")

            print_and_fixRetract(GREEN + "Charging started!" + RESET)
            self.hard_stop_robot()

        # overgang: laden -> niet laden
        elif previous_charging == 1 and current_charging == 0:
            
            # self.publish_docking_status("DOCKING_DISABLED")
            print_and_fixRetract(YELLOW + "Charging disconnected!" + RESET)

            if self.chargeflag == 1:
                print_and_fixRetract(YELLOW + "Code bereikt" + RESET)
                self.hard_stop_robot()
                self.retry_docking_if_not_charging()

        # interne status updaten
        self.robot['Charging'] = current_charging
        

    def update_battery_percentage(self):
        #batterijpercentage updaten en op topic zetten
        voltage = self.robot['Voltage']

        # formule voor 25V batterij
        percent = int(((voltage - 20) / 5) * 100)

        # clamp tussen 0 en 100
        percent = max(0, min(100, percent))

        # alleen publiceren als het veranderd is
        # if percent != self.last_battery_percent:

        #     msg = Int8()
        #     msg.data = percent

        #     for _ in range(3):
        #         self.Battery_pub.publish(msg)
        #         time.sleep(0.02)

        #     self.last_battery_percent = percent

        #     print_and_fixRetract(f"Battery percentage: {percent}%")



    def RED_Flag_callback(self, topic):

        # callback voor IR sensor
 
        # print_and_fixRetract(
        #     CYAN + f"[DEBUG] RED callback | topic={topic.data} | prev_RED={self.robot['RED']} | start_turn={self.start_turn} | find_red={self.find_redsignal}" + RESET
        # )

        self.red_count = topic.data
    
        # Als de robot niet laad is het IR interesant
        if self.robot['Charging']==0:

            #als IR verdwijnt terwijl we zoeken
            if topic.data==0 and self.robot['RED']==1:
                if((self.get_clock().now()-self.lost_red_flag).to_msg()).sec>=2:
                    print_and_fixRetract(YELLOW+"Infrared signal lost."+RESET)
                self.lost_red_flag = self.get_clock().now()
    
            #infrared gevonden
            if topic.data==1 and self.robot['RED']==0:
                print_and_fixRetract(GREEN+"Infrared signal founded."+RESET) 

    # update interne statussen
        if topic.data>0:
            self.robot['RED']=1
        else:
            self.robot['RED']=0

        # Activeer wanneer robot aan het draaien is om IR signaal te zoeken

        if self.start_turn==1:

      
            # print_and_fixRetract(
            #     CYAN + f"[DEBUG] TURNING | RED={self.robot['RED']} | find_red={self.find_redsignal}" + RESET
            # )

            # Als IR gevonden tijdens draaien
            if self.robot['RED']==1:
                self.find_redsignal = self.find_redsignal + 1 
            else:
                # Reset counter als signaal wegvalt tijdens rotatie
                self.find_redsignal = 0

    def Position_Update_callback(self, topic):

        """
        Update van charging station positie via RViz / external marker input.
        Slaat positie op in JSON bestand en past docking offset toe.
        """

          # Extract positie uit incoming PoseStamped
        position_dic={'p_x':0, 'p_y':0, 'orien_z':0, 'orien_w':0 }
        position_dic['p_x']=topic.pose.position.x
        position_dic['p_y']=topic.pose.position.y
        position_dic['orien_z']=topic.pose.orientation.z
        position_dic['orien_w']=topic.pose.orientation.w

        # Offset vooruit zodat robot niet exact op marker stopt
        tmp_yaw = math.atan2(2*(position_dic['orien_w']*position_dic['orien_z']),1-2*(position_dic['orien_z']**2))
        diff_x = math.cos(tmp_yaw)
        diff_y = math.sin(tmp_yaw)
        position_dic['p_x'] = position_dic['p_x'] + diff_x*self.diff_point
        position_dic['p_y'] = position_dic['p_y'] + diff_y*self.diff_point

        # 角度偏移
        # Offset vooruit zodat robot niet exact op marker stopt
            # Kleine hoekcorrectie voor betere docking alignement

        tmp_angle = math.radians(self.diff_angle)
        new_yaw = (tmp_yaw+tmp_angle)/2
        position_dic['orien_z'] = math.sin(new_yaw)
        position_dic['orien_w'] = math.cos(new_yaw)

            # Save naar JSON file (persistent storage)

        with open(json_file, 'w') as fp:
            json.dump(position_dic, fp, ensure_ascii=False)
            print_and_fixRetract("New charging pile position saved.")
           
         # Reload latest data in memory

        with open(json_file,'r')as fp:
            self.json_data = json.load(fp)

        #发布最新的充电桩位置话题
        # self.Pub_Charger_marker(position_dic['p_x'], position_dic['p_y'], position_dic['orien_z'], position_dic['orien_w'])

    

    def Odom_callback(self, topic):


        """
        Odometry callback.
        Bewaart huidige robotpositie (Z-rotatie / pose tracking).
        """



        '''更新的机器人实时位姿'''
        self.robot['Rotation_Z']=topic.pose.pose.position.z  

    def hard_stop_robot(self):

        """
        Emergency stop:
        - Stops rotation search
        - Publishes zero velocity multiple times to guarantee stop
        """
            

        # Stop zoek-rotatie
        self.start_turn = 0
        self.find_redsignal = 0

        # Publiceer expliciet 0-velocity
        stop = Twist()
        for _ in range(25):
            self.Cmd_vel_pub.publish(stop)
            time.sleep(0.05)

    def Stop_Charge(self, drive_forward=True):
        """

        Stop volledige charging flow:
        - Stops IR search
        - Optionally drives robot slightly forward to clear dock
        - Resets charging state
        """


        # print_and_fixRetract(
        #     CYAN + f"[DEBUG] STOP CHARGE BEFORE RESET | start_turn={self.start_turn} | find_red={self.find_redsignal} | RED={self.robot['RED']}" + RESET
        # )

        self.lost_power_once=1
        
        
        if drive_forward:
            print_and_fixRetract(
                    YELLOW + "Tijd voor naar voor te rijden om terug te starten aan normale gedrag" + RESET
                )
            move = Twist()
            move.linear.x = 0.20
            self.Cmd_vel_pub.publish(move)
            time.sleep(3.0)
            
            # Stop weer na het vooruit rijden
            move.linear.x = 0.0
            self.Cmd_vel_pub.publish(move)
        else:
            print_and_fixRetract(YELLOW + "Gevraagd om niet naar voren te rijden." + RESET)


        #reset charge mode
        self.chargeflag=0
        self.Pub_Recharger_Flag()
        
        print_and_fixRetract(
            CYAN + f"[DEBUG] STOP CHARGE AFTER RESET | start_turn={self.start_turn} | find_red={self.find_redsignal} | RED={self.robot['RED']}" + RESET
        )

        # reset van de code na een stop_charge()
        print_and_fixRetract(YELLOW + "Systeem gaat 5 seconden in rust voor herstart..." + RESET)
        self.must_reset = True
                

    def retry_docking_if_not_charging(self):
        """
        als robot faalt bij docken, dan proberen we opnieuw
        """
        
        # self.publish_docking_status("DOCKING_DISABLED")


        # Scherm publiceren van robotdocking
        self.publish_rpi_event("RobotDocking")

        print_and_fixRetract(
                YELLOW + "Time delay voor retry docking ingezet (verwacht stilstaan)" + RESET
            )
        time.sleep(2)

    
        print_and_fixRetract(
                YELLOW + "Niet aan het laden na hard stop, probeer opnieuw te docken..." + RESET
            )

        # Klein stukje vooruit
        move = Twist()
        move.linear.x = 0.20
        self.Cmd_vel_pub.publish(move)
        time.sleep(3.0)

        # Stop opnieuw
        self.Cmd_vel_pub.publish(Twist())

        # self.publish_docking_status("DOCKING_ENABLED")


        # Start opnieuw IR-zoekactie
        self.start_turn = 1
        self.find_redsignal = 0
        self.nav_end_z = self.robot['Rotation_Z']

        rotate = Twist()
        rotate.angular.z = 0.2
        self.Cmd_vel_pub.publish(rotate)


    def start_forced_charging(self):
        """Start charging logic (same as pressing Q).
        De robot wordt buiten deze code om al voor het laadstation gepositioneerd,
        dus er wordt niet meer genavigeerd: er wordt enkel gecontroleerd of er al
        een sterk IR-signaal is, en anders start het ronddraaien om te zoeken.
        """

        # indien meteen vanaf de locatie sterke IR wordt gezien
        if self.red_count >= 3:
            self.chargeflag = 1
            # self.publish_docking_status("DOCKING_ENABLED")
            self.Pub_Recharger_Flag()
            print_and_fixRetract(
                'Geforceerd laden: sterke IR gedetecteerd, direct docken.'
            )
        # nog geen sterk IR: begin met ronddraaien om te zoeken
        else:
            self.nav_end_z = self.robot['Rotation_Z']
            self.start_turn = 1
            self.find_redsignal = 0

            topic = Twist()
            topic.angular.z = 0.2
            self.Cmd_vel_pub.publish(topic)

            print_and_fixRetract(
                'Geforceerd laden: nog geen sterk IR-signaal, start ronddraaien om te zoeken.'
            )

    def autoRecharger(self, key):
        '''键盘控制开始自动回充:1-导航控制寻找充电桩,2-纯回充装备控制寻找充电桩
        '''


        """
        Main control loop (keyboard + autonomous charging logic)

            Handles:
            - Keyboard input (Q/E/T/Y)
            - Low battery detection
            - IR docking logic
            - Charging monitoring
        """


        # DETECTEREN OF LADEN VOLTOOID IS 
        if self.robot['Charging']==1:
            if (self.robot['Type']=='Plus'and self.robot['Voltage']>25) or (self.robot['Type']=='Mini' and self.robot['Voltage']>12.5):
                self.charge_complete=self.charge_complete+1  # verhoog counter bij elke itteratie dat batterij aan "vol" conditie voldoet
            else:
                self.charge_complete=0

        #forceer via keys te laden
        if key=='q' or key=='Q':
            self.start_forced_charging()
            

        #forceer via keys te stoppen
        elif key=='e' or key=='E':
            print_and_fixRetract('停止寻找充电桩或停止充电.(Stop finding charging pile or charging.)')
            self.Stop_Charge(drive_forward=True)

        # 测试用
        elif key=='t' or key=='T':
            self.set_charge_mode(1)
        elif key=='y' or key=='Y':
            self.set_charge_mode(0)

        #indien robot niet aan het laden is, kijk of spanning te laag is
        if self.robot['Charging']==0:
            if (self.robot['Type']=='Plus'and self.robot['Voltage']<0.0) or (self.robot['Type']=='Mini' and self.robot['Voltage']<0.0):
                time.sleep(1)
                self.power_lost_count=self.power_lost_count+1 # 低电量滤波

                # vaak genoeg te laag gedetecteerd?
                if self.power_lost_count>5 and self.lost_power_once==1:
                    self.power_lost_count=0

                    #  indien vaak genoeg te laaG en de robot niet aan het laden is : battery low produceren
                    if self.chargeflag==0:
                        # ROBOT HEEFT TE LAGE SPANNING : BT moet aan zet zijn, NIET DEZE CODE
                        
                        self.lost_power_once=0

            
                        # IDEE : ZET HIER CODE DIE IN EEN FILE SCHRIJFT DAT SPANNING TE LAAG IS. BT ZAL HIERNA BEPALEN WAT DE SITUATIE IS
                        self.write_battery_status("BATTERY-LOW")
    
            else:
                self.power_lost_count=0     

        # #频率1hz的循环任务
        if ((self.get_clock().now()-self.last_time).to_msg()).sec>=1:

            # Publiceer (indien nodig) het nieuwe batterijpercentage
            self.update_battery_percentage()


            #voor rviz2 laadstaion te tonen
            self.Pub_Charger_marker(
                self.json_data['p_x'], 
                self.json_data['p_y'], 
                self.json_data['orien_z'], 
                self.json_data['orien_w'])
            self.last_time=self.get_clock().now()

            #充电期间打印电池电压、充电时间
            if self.robot['Charging']==1:

                self.lost_power_once=1
                percent=0
                percen_form=0
                #self.hard_stop_robot() #harde stop van robot als hij aan het laden is
                #self.retry_docking_if_not_charging()
                if self.robot['Type']=='Plus':
                    percent= (self.robot['Voltage']-20)/5 
                    percent_form=format(percent, '.0%')
                if self.robot['Type']=='Mini':
                    percent= (self.robot['Voltage']-10)/2.5
                    percent_form=format(percent, '.0%')
                print_and_fixRetract("Robot is charging.")
                print_and_fixRetract("Robot battery: "+str(round(self.robot['Voltage'], 2))+"V = "+str(percent_form)+
                                     ", Charging current: "+str(round(self.robot['Charging_current'], 2))+"A.")
                mAh_time=0
                try:
                    mAh_time=1/self.robot['Charging_current']/1000
                except ZeroDivisionError:
                    pass
                left_battery=round(self.robot['BatteryCapacity']*percent, 2)
                if percent<1:
                    need_charge_battery=self.robot['BatteryCapacity']-left_battery
                    need_percent_form=format(1-percent, '.0%')      
                    print_and_fixRetract(str(self.robot['BatteryCapacity'])+"mAh*"+str(need_percent_form)+"="+str(need_charge_battery)+"mAh need to be charge, "+
                                         "cost "+str(round(need_charge_battery*mAh_time, 2))+" hours.")
                else:   
                    print_and_fixRetract(GREEN+"Robot battery is full."+RESET)
                print_and_fixRetract("\n")

         # bij rotatie IR gevonden : stop rotatie en ga IR docken
        if self.find_redsignal>=3:

            # print_and_fixRetract(
            #     GREEN + f"[DEBUG] DOCKING TRIGGERED | find_red={self.find_redsignal}" + RESET
            # )
            self.find_redsignal = 0 
            self.start_turn=0
            print_and_fixRetract(GREEN+'已通过自转发现红外信号,开始对接充电.(Infrared signals have been detected by rotation. Docking and charging has begun.)'+RESET)
            vel_topic=Twist()
            self.Cmd_vel_pub.publish(vel_topic) # 停止运动
            self.chargeflag=1 # 开启自动回充
            self.Pub_Recharger_Flag()

        # geen IR bij rotatie?
        if self.start_turn == 1:
            if abs(self.robot['Rotation_Z']-self.nav_end_z)>2*PI:
                self.start_turn=0
                self.Stop_Charge(drive_forward=True)
                print_and_fixRetract(RED+'自转已完成,无法找到充电桩位置,已停止自动回充.(Rotation completed, unable to locate charging station, automatic recharging has been stopped.)'+RESET)
                #BT docking gefaald, wel aan laadstation geraakt
                self.publish_event("DOCKING-FAILED")


        # kijken of laden compleet is
        if self.charge_complete>10:
            self.charge_complete=0
            if self.last_charge_complete!=0:
                self.last_charge_complete=0
            print_and_fixRetract(GREEN+'充电已完成.(Chrge complete.)'+RESET)#Charging complete
            
            if not self.battery_full_sent:
                self.battery_full_sent = True
                self.write_battery_status("BATTERY-OK")  #HIER LOGICA ZETTEN OM NAAR FILE TE SCHRIJVEN

            #BT charging gelukt
            # if not self.battery_full_sent:
            #     self.publish_event("BATTERY-FULL")
            #     self.battery_full_sent = True

        self.last_charge_complete=self.charge_complete


def main():
    rclpy.init()
    try:
        while rclpy.ok():
            autorecharger = AutoRecharger() 


            print_and_fixRetract(autorecharger.tips)
            
            # Initialiseer chassis instellingen
            tmp_sec = Int8()
            tmp_sec.data = 1
            tmp_vel = Twist()
            autorecharger.robot_security_off_pub.publish(tmp_sec)
            autorecharger.Cmd_vel_pub.publish(tmp_vel)
            
            # 2. Parameters laden (moet elke keer opnieuw voor een schone start)
            autorecharger.declare_parameter('robot_BatteryCapacity', 5000)
            autorecharger.declare_parameter('car_mode', "mini_mec")
            autorecharger.declare_parameter('diff_point', 1.2)
            autorecharger.declare_parameter('diff_angle', -15)

            with open(yaml_file, 'r') as file:
                params = yaml.safe_load(file)

            autorecharger.robot['BatteryCapacity'] = params['robot_info']['BatteryCapacity']
            autorecharger.robot['car_mode']        = params['robot_info']['car_mode']
            autorecharger.diff_point = params['robot_info']['diff_point']
            autorecharger.diff_angle = params['robot_info']['diff_angle']

            if autorecharger.robot['car_mode'][0:4] != 'mini':
                autorecharger.robot['Type'] = 'Plus'
            else:
                autorecharger.robot['Type'] = 'Mini'
            
            # 3. De 'Inner Loop': De actieve runtime van de robot
            should_quit_completely = False


            while rclpy.ok():
                key = get_key(settings) 
                autorecharger.autoRecharger(key) 
                
                rclpy.spin_once(autorecharger, timeout_sec=0.1)

                # Als Stop_Charge() is aangeroepen, breken we de Inner Loop
                if autorecharger.must_reset:
                    print_and_fixRetract(YELLOW + "Reset aangevraagd. Node wordt afgesloten..." + RESET)
                    break

                # Ctrl+C afhandeling
                if (key == '\x03'):
                    should_quit_completely = True
                    break
            
            # 4. Cleanup van de huidige sessie
            # Stop de motoren voor de zekerheid
            stop_msg = Twist()
            autorecharger.Cmd_vel_pub.publish(stop_msg)

            autorecharger.destroy_node()

            # Als de gebruiker Ctrl+C drukte, stoppen we de Outer Loop ook
            if should_quit_completely:
                print_and_fixRetract('Programma volledig afgesloten door gebruiker.')
                break

            # 5. De 10 seconden "echte" ruststand
            print_and_fixRetract(BLUE + "Systeem is nu 5 seconden inactief..." + RESET)
            time.sleep(5)
            print_and_fixRetract(GREEN + "Herstarten..." + RESET)

    except Exception as e:
        print_and_fixRetract(f"Er is een fout opgetreden: {e}")
    
    finally:

        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        rclpy.shutdown()

    print('Over and out.')
