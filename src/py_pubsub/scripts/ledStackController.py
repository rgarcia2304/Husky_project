import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from interface.msg import HuskyStateStatus
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

class ledStackController(Node):

    def __init__(self):
        
        super().__init__('hardware_node')
        self.status_callback = self.create_subscription(HuskyStateStatus,'status_alert',self.status_callback,10)
        self.physical_hardware = relay_controls()
        self.frequency= 1
        #Colors for light are 1 red, 2 orange, 4 green
        self.color = 0
        self.light_behavior = "none"

    def status_callback(self,msg):
        #conditions that make the husky robot invalid
        if msg.lidar_frequency_validator==False:
            self.frequency = 1
            self.color = 1
            self.light_behavior = "blink"
            self.set_light_behavior()
            return

        if  msg.battery_isok == False:
            self.frequency = 1
            self.color = 1
            self.light_behavior = "blink"
            self.set_light_behavior()
            return
    
        if msg.solution_mode_validator == False:
            self.frequency = .25
            self.color = 2
            self.light_behavior = "solid"
            self.set_light_behavior()
            return
        
        if msg.position_status == False:
            self.color = 1
            self.light_behavior = "solid"
            self.set_light_behavior()
            return

        #blink orange while we wait for status type to become valid
        if msg.status_type_validator==False:
            self.color = 2
            self.frequency = 0.5
            self.light_behavior = "solid"
            self.set_light_behavior()
            return
        
        #condition for battery low but no other errors
        if  msg.battery_isok == False:
            self.frequency = 1
            self.color = 1
            self.light_behavior = "solid"
            self.set_light_behavior()
            return

        #conditon for if everything is ok but bag is not recording
        if msg.is_recording ==False:
            self.frequency =1
            self.color= 4
            self.light_behavior= "blink"
            self.set_light_behavior()
            return 

        #if everything is ok just stay green
        self.color=4
        self.light_behavior="solid"
        self.set_light_behavior()
 
    def set_light_behavior(self):
        if self.light_behavior == "none":
            self.solid_light()

        if self.light_behavior == "blink":
            self.blink_light()
        
        if self.light_behavior =="solid":
            self.solid_light()

    def solid_light(self):
        time.sleep(1)
        self.physical_hardware.set_relay_state(self.color)
        
    def blink_light(self):
        time.sleep(self.frequency)
        self.physical_hardware.set_relay_state(self.color)
        time.sleep(self.frequency)
        self.physical_hardware.set_all_relays(False)
        
    def light_runner(self):
        while rclpy.ok():
            self.set_light_behavior()
            rclpy.spin_once(self,timeout_sec=0.1)

            
def main(args=None):
    rclpy.init(args=args)
    hardware_sensor = ledStackController()
    hardware_sensor.physical_hardware.open_ports('/dev/ttyACM0', 9600,1)
    hardware_sensor.physical_hardware.get_sw_version()
    hardware_sensor.physical_hardware.set_all_relays(True)
    hardware_sensor.light_runner()
    hardware_sensor.destroy_node()
    hardware_sensor.physical_hardware.close_ports()
    hardware_sensor.physical_hardware.set_all_relays(False)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        


