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

    def status_callback(self,msg):
        self.get_logger().info(f'Current Status: {msg.lidar_frequency_validator}')

        #FLASH LIGHT IF IT REACHES FALSE CONDITION TEST FOR LIDAR
        if msg.lidar_frequency_validator == False:
            self.physical_hardware.set_relay_state(1)
            time.sleep(.05)
            self.physical_hardware.set_all_relays(False)
        else:
            self.physical_hardware.set_relay_state(2)
            time.sleep(0.05)
            self.physical_hardware.set_all_relays(False)
            
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




        


