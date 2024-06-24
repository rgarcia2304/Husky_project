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
        self.frequency= 1
        #Colors for light are 1 red, 2 orange, 4 green
        self.color = 0
        self.action = "none"

    def status_callback(self,msg):

        #establish conditons and set variables here
        if msg.lidar_frequency_validator==False:
            self.frequency = 1
            self.color = 1
            self.action = "blink"
            self.light_controller()

        if  msg.battery_isok == False:
            self.frequency = 1
            self.color = 1
            self.action = "set_light_color"
            self.light_controller()
        else:
            self.frequency = 4
            self.color = 1
            self.action = "set_light_color"
            self.light_controller()

    def light_controller(self):
        if self.action == "none":
            self.set_light_color()
        if self.action == "blink":
            self.blink_light()
        
        if self.action =="set_light_color":
            self.set_light_color()

    def set_light_color(self):
        time.sleep(1)
        self.physical_hardware.set_relay_state(self.color)
        
    def blink_light(self):
        #color setting operations and blinking frequency
        time.sleep(self.frequency)
        self.physical_hardware.set_relay_state(self.color)
        time.sleep(self.frequency)
        self.physical_hardware.set_all_relays(False)
        
    def light_runner(self):
        while rclpy.ok():
            self.light_controller()
            rclpy.spin_once(self,timeout_sec=0.1)

            
def main(args=None):
    rclpy.init(args=args)
    hardware_sensor = Physical_Sensor()
    hardware_sensor.physical_hardware.open_ports('/dev/ttyACM2', 9600,1)
    hardware_sensor.physical_hardware.get_sw_version()
    hardware_sensor.physical_hardware.set_all_relays(True)
    hardware_sensor.light_runner()
    hardware_sensor.destroy_node()
    hardware_sensor.physical_hardware.close_ports()
    hardware_sensor.physical_hardware.set_all_relays(False)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        


