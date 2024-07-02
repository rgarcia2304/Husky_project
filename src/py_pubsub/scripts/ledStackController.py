import os
import sys

# Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the current directory to the Python path
sys.path.append(current_dir)

# Now you can import the module
import time
import rclpy
from rclpy.node import Node
from interface.msg import HuskyStateStatus
import relayControlsAPI


class ledStackController(Node):

    def __init__(self):
        
        super().__init__('hardware_node')
        self.status_subscriber = self.create_subscription(HuskyStateStatus,'status_alert',self.status_callback,10)
        self.physical_hardware = relayControlsAPI.relay_controls()
        self.led_stack_status = {"red":0.0,"orange":0.0,"green":0.0}
        self.old_led_stack_status= {"red":None,"orange":None,"green":None}
        self.sensor_status= HuskyStateStatus()
        self.prev_cmd = {"red":False,"orange":False,"green":False}
        self.ledStackDecoder={"red":1, "orange":2,"green":4}
        # self.current_code_color=0
        # self.max_frequency=0

        # self.is_red_blinking = False
        # self.is_orange_blinking = False
        # self.is_green_blinking= False

        # self.red_mult=0
        # self.orange_mult=0
        # self.green_mult=0

    def status_callback(self,msg):
         self.sensor_status= msg

    def setLedStackFromSensorState(self):
        #check to read for all the invalid HUSKY states
        if self.steady_red_light_messages():
                self.led_stack_status["red"]=2.0

        #check for status type message Validity
        if self.orange_blinking_light_messages():
            self.led_stack_status["orange"]= 0.5

        #check for HUSKY bag recording message validity
        if self.green_blinking_light_messages():
             self.led_stack_status["green"]=0.5
        
        #check for validity of all states
        if self.green_blinking_light_messages():
                self.led_stack_status["green"]= 2.0

    def steady_red_light_messages(self):
        if (self.sensor_status.lidar_frequency_validator==False or
            self.sensor_status.battery_isok == False or
            self.sensor_status.solution_mode_validator == False or
            self.sensor_status.position_status == False ):
            return True
        return False
    
    def orange_blinking_light_messages(self):
        if self.sensor_status.status_type_validator==False:
            return True
        return False
    
    def green_blinking_light_messages(self):
        if (self.sensor_status.is_recording ==False):
            return True
        return False
    
    def steady_green_light_messages(self):
        if (self.sensor_status["red"] <0.1 and
            self.sensor_status["orange"] <0.1 and
            self.led_stack_status["green"] <2.0):
            return True
        return False
    
    # def led_interact(self):
    #     #background controls
    #     counter=0

    #     red_blinking_frequency=None
    #     orange_blinking_frequency=None
    #     green_blinking_frequency=None

    #     if self.led_stack_status["red"] >0.9 or self.led_stack_status["red"]<0.1:
        #     self.is_red_blinking= False
        # else:
        #     self.is_red_blinking= True
        #     red_blinking_frequency = 1/(self.led_stack_status["red"])
        
        # if self.led_stack_status["orange"] >0.9 or self.led_stack_status["orange"]<0.1:
        #     self.is_orange_blinking= False
        # else:
        #     self.is_orange_blinking= True
        #     orange_blinking_frequency = 1/(self.led_stack_status["orange"])

        # if self.led_stack_status["green"] >0.9 or self.led_stack_status["green"]<0.1:
    #         self.is_green_blinking= False
    #     else:
    #         self.is_green_blinking= True
    #         green_blinking_frequency = 1/(self.led_stack_status["green"])
        
    #     self.max_frequency = max(green_blinking_frequency,orange_blinking_frequency,red_blinking_frequency)

    #     if self.is_red_blinking:
    #         self.red_mult= self.max_frequency/ self.is_red_blinking
        
    #     if self.is_orange_blinking:
    #         self.orange_mult = self.max_frequency/self.is_orange_blinking
         

    def blink_light(self,color):
        time.sleep(self.led_stack_status[color])
        self.physical_hardware.set_relay_state(self.ledStackDecoder[color])
        time.sleep(self.led_stack_status[color])
        self.physical_hardware.set_all_relays(False)     


    def controlLedFromStatus(self):
        for color in self.led_stack_status:

            #turns light color off from on state on LED STACK
            if self.led_stack_status[color] <0.1 and self.prev_cmd[color] == True:
                self.physical_hardware.set_relay_state(self.ledStackDecoder[color])
                self.prev_cmd[color]= False

            #turns light color on from off
            elif self.led_stack_status[color] >=0.9 and self.prev_cmd[color]== False:
                 self.physical_hardware.set_relay_state(self.ledStackDecoder[color])
                 self.prev_cmd[color] = True
            
            #turns blinking light on 
            else:
                self.prev_cmd[color] = True
                self.blink_light(color)
            
        
    def light_runner(self):
        while rclpy.ok():
            self.setLedStackFromSensorState()
            self.controlLedFromStatus()

            rclpy.spin_once(self,timeout_sec=0.25)

            
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


        


