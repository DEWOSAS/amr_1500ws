#!/usr/bin/env python3
import os
import pandas as pd
from pyModbusTCP.client import ModbusClient
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool, Int16, Int8
from facobot_msg.msg import BatteryStatus, PlcStatus
from facobot_module.srv import SafetyConfig

class plc_module(Node):
    def __init__(self):
        self.init_node()
        self.init_param()
        self.init_service()
        self.init_modbus()
        self.init_module(self.module)
        self.init_pub()
        self.init_sub()
        self.init_output()

        # เหมือน ROS1: ~4 Hz
        self.timer = self.create_timer(0.25, self.main_loop)

        self.complete_msg = String()
        self.complete_msg.data = "Complete"
        self.error_msg = String()

    def init_node(self):
        super().__init__("modbus_plc")
        self.get_logger().info("Node : modbus_plc")

    def init_param(self):
        # คงพารามิเตอร์ไว้ แต่จะไม่ใช้ simulate_offline
        self.declare_parameter("modbus_server_ip", "192.168.1.3")
        self.declare_parameter("modbus_init_output", "00000000")
        self.declare_parameter("battery_level_warning", 90.0)
        self.declare_parameter("battery_level_alarm", 30.0)
        self.declare_parameter("module", "conveyor")
        self.declare_parameter("module_timeout", 15.0)

        self.modbus_server_ip = self.get_parameter("modbus_server_ip").get_parameter_value().string_value
        self.battery_level_warning = self.get_parameter("battery_level_warning").get_parameter_value().double_value
        self.battery_level_alarm = self.get_parameter("battery_level_alarm").get_parameter_value().double_value
        self.module = self.get_parameter("module").get_parameter_value().string_value
        self.module_timeout = self.get_parameter("module_timeout").get_parameter_value().double_value * 4
        self.modbus_init_output = self.get_parameter("modbus_init_output").get_parameter_value().string_value

        # QoS depth = 3 ให้คล้าย ROS1 queue_size=3
        self.pub_disable_safety = self.create_publisher(Int8, "/disable_safety", 3)

        self.facobot_core_path = get_package_share_directory('facobot_core')
        self.safety_config_path = os.path.join(self.facobot_core_path, 'profile', 'safety_config.csv')

        # battery
        self.battery_status = BatteryStatus()
        self.battery_status.battery_level.data = 0.0
        self.battery_status.battery_current.data = 0.0

        # plc status
        self.plc_status = PlcStatus()
        self.plc_status.mode_msg = String()
        self.plc_status.emer_msg = Bool()
        self.plc_status.mode_msg.data = "Manual"
        self.plc_status.emer_msg.data = False
        self.plc_status.din_msg = [Bool() for _ in range(16)]
        self.plc_status.dout_msg = [Bool() for _ in range(16)]
        self.plc_status.plc_register_msg = [Int16() for _ in range(4)]

        # io states
        self.dout_key = [f"dout_{i}" for i in range(1, 17)]
        self.dout_target_state = [False]*16
        self.dout_latest_state = [False]*16
        self.din_key = [f"din_{i}" for i in range(1, 17)]
        self.din_latest_value = [False]*16

        self.sound_latest_state = [False, False, False]
        self.sound_target_state = [False, False, False]
        self.sound_list = {
            "none":[False,False,False],
            "sound1":[True,False,False],
            "sound2":[False,True,False],
            "sound3":[True,True,False],
            "sound4":[False,False,True],
            "sound5":[False,False,False],
            "sound6":[False,False,False],
            "sound7":[False,False,False]
        }
        self.sound_map = {"none":"none", "working":"sound1", "warning":"sound2", "emer":"sound3"}

        self.led_target_state = [False,False,False]
        self.led_latest_state = [False,False,False]

        self.charge_contact_target_state = False
        self.charge_contact_latest_state = False
        self.led_charge_target_state = False
        self.led_charge_latest_state = False
        self.led_docking_target_state = False
        self.led_docking_latest_state = False
        self.led_warning_target_state = False
        self.led_warning_latest_state = False

        self.mode = False
        self.emer = False
        self.reset = False
        self.cancel = False
        self.shutdown = False
        self.mode_list = ["Auto","Manual","Reset"]

        self.now_time = self.get_clock().now().nanoseconds // 10**9
        self.last_time = self.now_time
        self.module_count = 0

        self.battery_level = 50.0
        self.battery_current = 0.0
        self.count = 0

        self.module_state = "standby"
        self.module_io_map = {}
        self.conveyor_dir = 0
        self.send_cancel = True
        self.send_reset = True

    def init_modbus(self):
        # เหมือน ROS1: ให้ lib เปิด/ปิดเองทุกคำสั่ง
        self.plc_modbus = ModbusClient(
            host=self.modbus_server_ip,
            port=502,
            auto_open=True,
            auto_close=True
        )
        self.modbus_addr = {
            "led_charge":8278,
            "led_docking":8279, 
            "led_warning":8280, 
            "led_low_battery":8281,
            
            "sound_bit1":8282, 
            "sound_bit2":8283, 
            "sound_bit3":8284,
            
            "led_r":8285, 
            "led_g":8286, 
            "led_b":8287,
            
            "charge_contact":8288, 
            "reset":8289,
            "emer":8290, 
            "mode":8291, 
            "shutdown":8292,
            
            "din_1":8293, 
            "din_2":8294, 
            "din_3":8295, 
            "din_4":8296, 
            "din_5":8297, 
            "din_6":8298,
            "din_7":8299, 
            "din_8":8300, 
            "din_9":8301, 
            "din_10":8302, 
            "din_11":8303, 
            "din_12":8304,
            "din_13":8305, 
            "din_14":8306, 
            "din_15":8307, 
            "din_16":8308,
            
            "dout_1":8309, 
            "dout_2":8310, 
            "dout_3":8311, 
            "dout_4":8312, #turn
            
            "dout_5":8313, #turn
            
            "dout_6":8314,
            "dout_7":8315, 
            "dout_8":8316, 
            "dout_9":8317, #battery
            
            "dout_10":8318, #battery 
            
            "dout_11":8319,
            "dout_12":8320, 
            "dout_13":8321, 
            "dout_14":8322, 
            "dout_15":8323, 
            "dout_16":8324,
            
            "cancel":8325
        }

    def init_module(self, module=""):
        if module == "pallet":
            self.module_state = "standby"
            self.module_io_map = {
                "power":self.dout_key.index("dout_1"),
                "direction":self.dout_key.index("dout_2"),
                "left_1":self.din_key.index("din_1"),
                "left_2":self.din_key.index("din_2"),
                "left_3":self.din_key.index("din_3"),
                "left_4":self.din_key.index("din_4"),
                "right_1":self.din_key.index("din_5"),
                "right_2":self.din_key.index("din_6"),
                "right_3":self.din_key.index("din_7"),
                "right_4":self.din_key.index("din_8")
            }
        elif module == "conveyor":
            self.module_state = "standby"
            self.module_io_map = {
                "power":self.dout_key.index("dout_1"),
                "direction":self.dout_key.index("dout_4"),
                "front_ir":self.din_key.index("din_2"),
                "back_ir":self.din_key.index("din_3")
            }
        elif module == "towing":
            self.module_state = "home"
            self.towing_state = ["fix_tail","free_cart"]
            self.module_io_map = {
                "grip_cart":self.dout_key.index("dout_6"),
                "fix_tail":self.dout_key.index("dout_7"),
                "hold":self.dout_key.index("dout_15"),
                "amr_mode":self.dout_key.index("dout_8"),
                "turn_left_led":self.dout_key.index("dout_4"),
                "turn_right_led":self.dout_key.index("dout_5"),
                "cart_detect_r":self.din_key.index("din_13"),
                "cart_detect_l":self.din_key.index("din_14"),
                "fix_tail_complete":self.din_key.index("din_9"),
                "free_tail_complete":self.din_key.index("din_10"),
                "grip_cart_complete":self.din_key.index("din_11"),
                "free_cart_complete":self.din_key.index("din_12")
            }

    def load_config(self):
        conf = {}
        if os.path.exists(self.safety_config_path):
            df = pd.read_csv(self.safety_config_path, header=0, encoding='utf-8-sig', na_filter=False)
            self.ParameterConfig = df[['ParameterCode','Value']]
            conf = {x[0]:x[1] for x in self.ParameterConfig.itertuples(index=False)}
        for name in conf:
            if name == "running_sound":
                self.sound_map["working"] = conf[name]
            elif name == "warning_sound":
                self.sound_map["warning"] = conf[name]
        self.get_logger().info(str(self.sound_map))

    def set_sound_profile(self, request, response):
        self.get_logger().info(f"Set Sound Profile => {request.cmd}")
        if request.cmd == 1:
            self.load_config()
            response.ack = request.cmd
        else:
            response.ack = 0
        return response

    def init_service(self):
        self.load_config()
        self.safetyconfig_srv = self.create_service(SafetyConfig,"PlcConfig",self.set_sound_profile)

    def init_pub(self):
        # QoS 3 ให้ใกล้ ROS1
        self.pub_status = self.create_publisher(PlcStatus, "/plc_status", 3)
        self.pub_battery = self.create_publisher(BatteryStatus, "/battery", 3)
        self.pub_module_status = self.create_publisher(String, "/module_status", 3)
        self.pub_error_report = self.create_publisher(String, "/error_report", 3)

    def init_sub(self):
        self.plccommand_sub = self.create_subscription(String, "/plc_command", self.plc_command_callback, 10)
        self.modulecommand_sub = self.create_subscription(String, "/module_command", self.module_command_callback, 10)

    # ====== CALLBACKS (เหมือน ROS1: ไม่ publish ทันที) =====
    def plc_command_callback(self, msg):
        cmd = msg.data.split(":")
        if cmd[0] == "DOUT":
            pin = cmd[1]
            state = int(cmd[2])
            pin = self.dout_key.index("dout_"+pin)
            self.dout_target_state[pin] = bool(state)

        elif cmd[0] == "LED":
            pattle = cmd[1]
            if pattle == "charge":
                self.charge_contact_target_state = True
                self.led_docking_target_state = True
                self.led_charge_target_state = False
            elif pattle == "discharge":
                self.charge_contact_target_state = False
                self.led_docking_target_state = False
                self.led_charge_target_state = False
            elif pattle == "warning":
                self.led_warning_target_state = True
            elif pattle == "normal":
                self.led_warning_target_state = False

        elif cmd[0] == "SOUND":
            self.sound_pattle_select(cmd[1])

    def module_command_callback(self, msg):
        state = msg.data
        self.get_logger().info(f"{self.module}:{self.module_state}:{state}")
        if self.module == "pallet":
            if state in ["lift_up","up"]:
                self.module_count = 0
                if self.module_state == "standby":
                    self.module_state = "direction_up"
            elif state in ["lift_down","down"]:
                self.module_count = 0
                if self.module_state == "standby":
                    self.module_state = "direction_down"

        elif self.module == "towing":
            self.module_count = 0
            pin = self.module_io_map["hold"]
            if not self.emer:
                self.dout_target_state[pin] = False

            if state == "home":
                if self.module_state != "grip":
                    self.module_state = "home"
                    self.towing_state = ["fix_tail","free_cart"]
                else:
                    self.emer = True
            elif state == "grip":
                self.module_state = "grip"
                self.towing_state = ["fix_tail","grip_cart"]
            elif state == "tow":
                self.module_state = "tow"
                self.towing_state = ["free_tail","grip_cart"]
            elif state == "free":
                if self.module_state != "grip":
                    self.module_state = "free"
                    self.towing_state = ["free_tail","free_cart"]
                else:
                    self.emer = True
            elif state == "cancel":
                self.module_state = "cancel"

        elif self.module == "conveyor":
            if state in ["up","left","load_in","load_in_l","load_in_r"]:
                self.module_count = 0
                if self.module_state == "standby":
                    self.module_state = "load_in"
                self.conveyor_dir = 1 if (state.endswith('r') or state == 'left') else 0
            elif state in ["down","right","load_out","load_out_l","load_out_r"]:
                self.module_count = 0
                if self.module_state == "standby":
                    self.module_state = "load_out"
                self.conveyor_dir = 1 if (state.endswith('r') or state == 'right') else 0
            elif state in ["left","cancel"]:
                self.module_count = 0
                self.module_state = "standby"
                pin_1 = self.module_io_map["power"]
                pin_0 = self.module_io_map["direction"]
                self.dout_target_state[pin_1] = False
                self.dout_target_state[pin_0] = False

    def sound_pattle_select(self, pattle):
        if pattle in self.sound_map:
            self.sound_target_state = self.sound_list[self.sound_map[pattle]]
        elif pattle in self.sound_list:
            self.sound_target_state = self.sound_list[pattle]

    def init_output(self):
        try:
            for num, state in enumerate(self.modbus_init_output):
                if num < len(self.dout_target_state):
                    self.dout_target_state[num] = int(state)
        except Exception:
            self.get_logger().error("Initial Output : Format not match!!")
        try:
            if self.dout_latest_state != self.dout_target_state:
                self.write_multiple_coils(self.modbus_addr["dout_1"], self.dout_target_state)
        except Exception:
            self.get_logger().error("Initial Output : Write io error!!")

    def tohex(self, val, nbits):
        return hex((val + (1 << nbits)) % (1 << nbits))

    def read_battery(self):
        # battery level
        try:
            time.sleep(0.008)
            val = self.plc_modbus.read_holding_registers(3016, 2)
            if val is not None:
                A_1 = self.tohex(val[1],16); B_1 = self.tohex(val[0],16)
                A_2 = str(A_1[2:]); B_2 = str(B_1[2:])
                A_3 = bytes.fromhex(A_2).decode("utf-8")
                B_3 = bytes.fromhex(B_2).decode("utf-8")
                capa = (int(A_3 + B_3, 16)/1000.0)*2
                if capa >= 70.0:   
                    self.led_target_state = [False,True,False]
                elif capa >= 30.0:
                    self.led_target_state = [True,True,True]
                else:               
                    self.led_target_state = [True,True,False]
                if self.battery_level != capa:
                    self.count += 1
                    if self.count > 3:
                        self.battery_level = capa
                else:
                    self.count = 0
                idx_warning = self.dout_key.index("dout_9")
                idx_alarm   = self.dout_key.index("dout_10")
                if self.battery_level < self.battery_level_alarm:
                    self.dout_target_state[idx_alarm] = True
                    self.dout_target_state[idx_warning] = False
                elif self.battery_level < self.battery_level_warning:
                    self.dout_target_state[idx_alarm] = False
                    self.dout_target_state[idx_warning] = False
                else:
                    self.dout_target_state[idx_alarm] = False
                    self.dout_target_state[idx_warning] = True
        except Exception:
            msg = String(); msg.data = "ER0003:PLC Lost Connection!!"
            self.pub_error_report.publish(msg)
            self.get_logger().error("PLC : Read battery level error!!")

        # battery current
        try:
            time.sleep(0.008)
            val = self.plc_modbus.read_holding_registers(3066,2)
            if val is not None:
                A_1 = self.tohex(val[1],16)
                B_1 = self.tohex(val[0],16)
                A_2 = str(A_1[2:])
                B_2 = str(B_1[2:])
                A_3 = bytes.fromhex(A_2).decode("utf-8")							# convert Hex to Acsii
                B_3 = bytes.fromhex(B_2).decode("utf-8")
                val =  int(A_3+B_3,16)
                # if (val & 32768) == 32768:
                #     current = (val - 65536)/10
                # else:
                current = val/10
            if current < 30.0:
                    self.battery_current = current
        except Exception:
            self.get_logger().error("PLC : Read battery current error!!")

        self.handle_charge_light()

    def handle_charge_light(self):
        self.now_time = self.get_clock().now().nanoseconds // 10**9
        diff_time = self.now_time - self.last_time
        if diff_time >= 2:
            self.last_time = self.now_time
            if self.battery_current >= 1 and self.battery_level < 100.0:
                self.led_charge_target_state = True
                self.led_docking_target_state = False
            else:
                self.led_charge_target_state = False

    def read_register(self):
        time.sleep(0.008)
        val = self.plc_modbus.read_holding_registers(310,4)
        if val is not None and len(val) == 4:
            for num in range(4):
                self.plc_status.plc_register_msg[num].data = val[num]

    # เหมือน ROS1: ไม่ต้อง ensure_plc/connect ทุกทิก
    def read_multiple_coils(self, addr, num):
        time.sleep(0.008)
        data = self.plc_modbus.read_coils(self.modbus_addr[addr], num)
        if data is None:
            self.get_logger().error(f"read multiple coils fail : {addr}, : {num}")
        return data

    def read_coil(self, addr):
        time.sleep(0.008)
        data = self.plc_modbus.read_coils(self.modbus_addr[addr], 1)
        if data is None:
            self.get_logger().error(f"read coil fail : {addr}")
            return data
        else:
            return data[0]

    def read_io_new(self):
        try:
            data = self.read_multiple_coils("led_charge",48)
            if data is not None:
                self.led_charge_latest_state  = data[0]
                self.led_docking_latest_state = data[1]
                self.led_warning_latest_state = data[2]
                self.sound_latest_state       = data[4:7]
                self.led_latest_state         = data[7:10]
                self.charge_contact_latest_state = data[10]
                self.reset    = data[11]
                self.emer     = data[12]
                self.mode     = data[13]
                self.shutdown = data[14]
                self.din_latest_value  = data[15:31]
                self.dout_latest_state = data[31:47]
                self.cancel  = data[47]
        except Exception:
            self.get_logger().error("PLC : Read io error!!")

    def handle_shutdown(self):
        if self.shutdown:
            self.get_logger().info("shutdown . . .")
            os.system("shutdown now -h")

    def handle_module(self):
        if self.module == "pallet":
            if self.module_state != "standby":
                l_detect = sum([
                    self.din_latest_value[self.module_io_map["left_1"]],
                    self.din_latest_value[self.module_io_map["left_2"]],
                    self.din_latest_value[self.module_io_map["left_3"]],
                    self.din_latest_value[self.module_io_map["left_4"]]
                ])
                r_detect = sum([
                    self.din_latest_value[self.module_io_map["right_1"]],
                    self.din_latest_value[self.module_io_map["right_2"]],
                    self.din_latest_value[self.module_io_map["right_3"]],
                    self.din_latest_value[self.module_io_map["right_4"]]
                ])

            if self.module_state == "direction_up":
                pin = self.module_io_map["direction"]
                self.dout_target_state[pin] = True
                if self.dout_target_state[pin] == self.dout_latest_state[pin]:
                    self.module_state = "lift_up"
            elif self.module_state == "direction_down":
                pin = self.module_io_map["direction"]
                self.dout_target_state[pin] = False
                if self.dout_target_state[pin] == self.dout_latest_state[pin]:
                    self.module_state = "lift_down"
            elif self.module_state == "lift_up":
                pin = self.module_io_map["power"]
                self.dout_target_state[pin] = True
                if self.dout_target_state[pin] == self.dout_latest_state[pin]:
                    if l_detect == 0 and r_detect == 0:
                        self.module_state = "stop"
            elif self.module_state == "lift_down":
                pin = self.module_io_map["power"]
                self.dout_target_state[pin] = True
                if self.dout_target_state[pin] == self.dout_latest_state[pin]:
                    if l_detect > 2 and r_detect > 2:
                        self.module_state = "stop"
            elif self.module_state == "stop":
                pin = self.module_io_map["power"]
                self.dout_target_state[pin] = False
                if self.dout_target_state[pin] == self.dout_latest_state[pin]:
                    self.module_state = "standby"
                    self.pub_module_status.publish(self.complete_msg)

            if self.module_state != "standby" and self.module_timeout > 0:
                self.module_count += 1
                if self.module_count > self.module_timeout:
                    self.module_count = 0
                    self.module_state = "stop"
                    self.get_logger().error(f"time_out : {self.module_timeout}")
                    self.pub_module_status.publish(String(data="Fail"))

        elif self.module == "towing":
            fix_tail_complete  = self.din_latest_value[self.module_io_map["fix_tail_complete"]]
            free_tail_complete = self.din_latest_value[self.module_io_map["free_tail_complete"]]
            grip_cart_complete = self.din_latest_value[self.module_io_map["grip_cart_complete"]]
            free_cart_complete = self.din_latest_value[self.module_io_map["free_cart_complete"]]
            cart_detect_l_current = self.din_latest_value[self.module_io_map["cart_detect_l"]]
            cart_detect_r_current = self.din_latest_value[self.module_io_map["cart_detect_r"]]

            if self.emer:
                self.module_state = "emer"
                pin = self.module_io_map["hold"]
                self.dout_target_state[pin] = True

            if self.module_state == "cancel":
                self.pub_module_status.publish(String(data="Cancel"))
                pin = self.module_io_map["fix_tail"]; self.dout_target_state[pin] = False
                pin = self.module_io_map["grip_cart"]; self.dout_target_state[pin] = False
                self.led_warning_target_state = False
                self.sound_pattle_select("none")
                self.module_state = "standby"

            elif self.module_state != "standby":
                complete_state = 0
                self.sound_pattle_select("working")
                if self.towing_state[0] == "fix_tail":
                    if self.module_state == "grip":
                        complete_state += 1
                    else:
                        pin = self.module_io_map["fix_tail"]; self.dout_target_state[pin] = True
                        if fix_tail_complete: complete_state += 1
                elif self.towing_state[0] == "free_tail":
                    pin = self.module_io_map["fix_tail"]; self.dout_target_state[pin] = False
                    if free_tail_complete: complete_state += 1

                if self.towing_state[1] == "grip_cart":
                    if self.module_state == "grip":
                        self.module_count = 0
                        if cart_detect_l_current and cart_detect_r_current:
                            self.led_warning_target_state = False
                            pin = self.module_io_map["grip_cart"]; self.dout_target_state[pin] = True
                            if grip_cart_complete:
                                self.sound_pattle_select("none")
                                complete_state += 1
                        else:
                            pin = self.module_io_map["grip_cart"]
                            if not self.dout_target_state[pin]:
                                self.sound_pattle_select("warning")
                                self.led_warning_target_state = True
                            else:
                                if grip_cart_complete:
                                    self.sound_pattle_select("none")
                                    complete_state += 1
                    else:
                        complete_state += 1
                elif self.towing_state[1] == "free_cart":
                    pin = self.module_io_map["grip_cart"]; self.dout_target_state[pin] = False
                    if free_cart_complete: complete_state += 1

                if complete_state == 2:
                    self.module_state = "standby"
                    self.pub_module_status.publish(self.complete_msg)
                    self.sound_pattle_select("none")

                if self.module_timeout > 0:
                    self.module_count += 1
                    if self.module_count > self.module_timeout:
                        self.module_count = 0
                        self.module_state = "cancel"
                        self.pub_module_status.publish(String(data="Fail"))
                        self.sound_pattle_select("none")

            # publish disable_safety แบบ Int8(data=…)
            if self.towing_state[0] == "fix_tail":
                if cart_detect_l_current and cart_detect_r_current:
                    self.pub_disable_safety.publish(Int8(data=11))
                elif cart_detect_l_current:
                    self.pub_disable_safety.publish(Int8(data=12))
                elif cart_detect_r_current:
                    self.pub_disable_safety.publish(Int8(data=13))
                else:
                    self.pub_disable_safety.publish(Int8(data=10))
            else:
                self.pub_disable_safety.publish(Int8(data=9))

        elif self.module == "conveyor":
            if self.conveyor_dir:
                back_tr_current  = self.din_latest_value[self.module_io_map["front_ir"]]
                front_tr_current = self.din_latest_value[self.module_io_map["back_ir"]]
            else:
                front_tr_current = self.din_latest_value[self.module_io_map["front_ir"]]
                back_tr_current  = self.din_latest_value[self.module_io_map["back_ir"]]

            pin_1 = self.module_io_map["power"]
            pin_0 = self.module_io_map["direction"]

            if self.module_state == "load_out":
                if (not front_tr_current) or (not back_tr_current):
                    self.dout_target_state[pin_1] = True
                    self.dout_target_state[pin_0] = True if self.conveyor_dir else False
                elif self.module_count < 12:
                    self.module_count += 1
                else:
                    self.dout_target_state[pin_1] = False
                    self.dout_target_state[pin_0] = False
                    self.module_state = "standby"
                    self.pub_module_status.publish(self.complete_msg)

            elif self.module_state == "load_in":
                if back_tr_current:
                    self.dout_target_state[pin_1] = True
                    self.dout_target_state[pin_0] = False if self.conveyor_dir else True
                else:
                    self.dout_target_state[pin_1] = False
                    self.dout_target_state[pin_0] = False
                    self.module_state = "standby"
                    self.pub_module_status.publish(self.complete_msg)

    def write_multiple_coils(self, addr, values):
        time.sleep(0.008)
        data = self.plc_modbus.write_multiple_coils(self.modbus_addr[addr], values)
        if data is None:
            self.get_logger().error(f"write multiple coils fail : {addr}, : {values}")
        return data

    def write_coil(self, addr, value):
        time.sleep(0.008)
        data = self.plc_modbus.write_single_coil(self.modbus_addr[addr], value)
        if data is None:
            self.get_logger().error(f"write coil fail : {addr}, : {value}")
        return data

    def write_io_new(self):
        try:
            state_io_target = [self.led_charge_target_state,
                               self.led_docking_target_state,
                               self.led_warning_target_state,
                               False]
            dout_io_target = state_io_target + self.sound_target_state + self.led_target_state + [self.charge_contact_target_state]
            self.write_multiple_coils("led_charge", dout_io_target)
            self.write_multiple_coils("dout_1", self.dout_target_state)
        except Exception:
            self.get_logger().error("PLC : Write io error!!")

    def handle_status(self):
        self.plc_status.emer_msg.data = self.emer

        if self.cancel:
            if self.send_cancel:
                self.pub_error_report.publish(String(data="CANCEL"))
                self.send_cancel = False
        else:
            self.send_cancel = True

        if self.reset:
            if self.send_reset:
                self.pub_error_report.publish(String(data="ER0001:Reset"))
                self.send_reset = False
        else:
            self.plc_status.mode_msg.data = self.mode_list[int(self.mode)]
            self.send_reset = True

        for num in range(16):
            self.plc_status.din_msg[num].data  = bool(self.din_latest_value[num])
            self.plc_status.dout_msg[num].data = bool(self.dout_latest_state[num])

        self.pub_status.publish(self.plc_status)

        self.battery_status.battery_level.data  = self.battery_level
        self.battery_status.battery_current.data = self.battery_current
        self.pub_battery.publish(self.battery_status)

    def main_loop(self):
        # เหมือน ROS1: ไม่มี connect_plc ทุกทิก
        self.read_battery()
        self.read_io_new()
        self.read_register()
        self.handle_shutdown()
        self.handle_module()
        self.write_io_new()
        self.handle_status()

def main(args=None):
    rclpy.init(args=args)
    node = plc_module()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
