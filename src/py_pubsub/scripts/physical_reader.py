import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from interface.msg import ErrorMsg
import tkinter as tk

class relay_controls():

    def __init__(self):
        self.ser = None

    def open_ports(self,port,baudRate,time_out):
        self.ser = serial.Serial(port, baudRate,timeout=time_out)

    def close_ports(self):
        self.ser.close
        
    def get_sw_version(self):
        num = 90
        self.ser.write(num.to_bytes(1,'big'))     # write a string
        time.sleep(1)
        version = self.ser.read(size=2)
        print(version)

    def set_relay_state(self,relay_num:int):
        relay_num.to_bytes(1,'big')
        self.ser.write(bytes([0x5C, relay_num]))
    
    def set_all_relays(self,state):
        num=0
        if state==True:
            num=100
            self.ser.write(num.to_bytes(1,'big'))

        elif state==False:
            num=110
            self.ser.write(num.to_bytes(1,'big'))
    
    def get_state(self,relay_num:int):
        relay_num.to_bytes(1,'big')
        self.ser.write(bytes([0x5B,relay_num]))
        current_state=self.ser.read(size=1)
        print(current_state)

class Physical_Sensor(Node):

    def __init__(self):
        
        super().__init__('hardware_node')
        self.status_callback = self.create_subscription(ErrorMsg,'status_alert',self.status_callback,10)
        self.physical_hardware = relay_controls()

        self.current_zpos_status = None
        self.current_xpos_status = None
        self.current_ypos_status = None
        self.current_rtk_status = None
        self.current_lidar_status = None
        self.status_type_status = None
        self.solution_mode_status = None

    def status_callback(self,msg):
        self.current_zpos_status = msg.z_pos
        self.current_xpos_status = msg.x_pos
        self.current_ypos_status = msg.z_pos
        self.current_rtk_status = msg.rtk_status
        self.current_lidar_status = msg.lidar_frequency_validator
        self.status_type_status = msg.status_type_validator
        self.solution_mode_status = msg.solution_mode_validator
        self.get_logger().info(f'Current Status: {msg.lidar_frequency_validator}')

        #FLASH LIGHT IF IT REACHES FALSE CONDITION
        if msg.lidar_frequency_validator == False:
            print('Hello')
            self.physical_hardware.set_relay_state(152)
            time.sleep(1)
            self.physical_hardware.set_all_relays(False)

            #self.physical_hardware.set_relay_state(10)
        #else:
            #self.physical_hardware.set_relay_state(110)
            #self.physical_hardware.set_relay_state(102)

def main(args=None):
    rclpy.init(args=args)
    hardware_sensor = Physical_Sensor()
    hardware_sensor.physical_hardware.open_ports('/dev/ttyACM0', 9600,1)
    hardware_sensor.physical_hardware.get_sw_version()
    hardware_sensor.physical_hardware.set_all_relays(True)
    rclpy.spin(hardware_sensor)
    hardware_sensor.destroy_node()
    hardware_sensor.physical_hardware.close_ports()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




        


