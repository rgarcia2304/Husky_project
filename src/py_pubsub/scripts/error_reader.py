
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from interface.msg import ErrorMsg
import tkinter as tk

class SensorReader(Node):
    def __init__(self):
        super().__init__('error_reader')        

        # Create subscribers
        self.topic_monitor = self.create_subscription(String,'/state_topic',self.monitor_callback,10)
        #rtk information
        self.rtk_status_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.rtk_status_monitor_callback,10)
        self.rtk_lidar_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.lidar_status_monitor_callback,10)
        self.localiation1_status_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.localization1_status_monitor_callback,10)
        self.localiation2_status_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.localization2_status_monitor_callback,10)
        self.xpos_status_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.xpos_monitor_callback,10)
        self.ypos_status_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.ypos_monitor_callback,10)
        self.zpos_status_reader = self.create_subscription(ErrorMsg,'temperature_alert',self.zpos_monitor_callback,10)

        self.current_zpos_status = None
        self.current_xpos_status = None
        self.current_ypos_status = None
        self.current_rtk_status = None
        self.current_lidar_status = None
        self.current_localization1_status = None
        self.current_localization2_status = None
        self.access_dictionary = None
        self.init_ui()

    def monitor_callback(self, msg):
        try:
            # Convert the JSON string back to a dictionary
            state_dict = json.loads(msg.data)
            self.access_dictionary = state_dict
            self.get_logger().info(f'Received state dictionary: {state_dict}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON string: {e}')

    #display information 
    def init_ui(self):
        self.root = tk.Tk()
        self.root.title("Error Display")
        self.canvas = tk.Canvas(self.root, width=500, height=100)
        self.canvas.pack()

        #rtk display
        self.rtk_light= self.canvas.create_oval(30, 30, 40, 40, fill="yellow")
        self.canvas.create_text(40, 80, text=f"rtk_status", anchor=tk.CENTER)
        #lidar light
        self.lidar_light= self.canvas.create_oval(60, 30, 70, 40, fill="yellow")
        self.canvas.create_text(140, 80, text=f"lidar_status", anchor=tk.CENTER)
        #localization check light 1
        self.localization_light1= self.canvas.create_oval(90, 30, 100, 40, fill="yellow")
        self.canvas.create_text(240, 80, text=f"localiaztion1", anchor=tk.CENTER)
        #localization check light2
        self.localization_light2= self.canvas.create_oval(120, 30, 130, 40, fill="yellow")
        self.canvas.create_text(340, 80, text=f"localization2", anchor=tk.CENTER)
        #x positon light
        self.xpos_light= self.canvas.create_oval(150, 30, 160, 40, fill="yellow")
        self.canvas.create_text(340, 80, text=f"localization2", anchor=tk.CENTER)
        #y positon light
        self.ypos_light= self.canvas.create_oval(180, 30, 190, 40, fill="yellow")
        self.canvas.create_text(340, 80, text=f"localization2", anchor=tk.CENTER)
        #y positon light
        self.zpos_light= self.canvas.create_oval(210, 30, 220, 40, fill="yellow")
        self.canvas.create_text(340, 80, text=f"localization2", anchor=tk.CENTER)
        #closing actions
        self.root.after(100, self.check_ros)
        self.root.mainloop()
        
    def zpos_monitor_callback(self,msg):
            if msg.z_pos==False:
                self.current_zpos_status= False
                if self.current_zpos_status == False:
                    self.update_zpos_status_light("red")
            else:
                self.update_zpos_status_light("green")
            self.get_logger().info(f'Received state {msg.z_pos}\n')
    
    def update_zpos_status_light(self, color):
        self.canvas.itemconfig(self.zpos_light, fill=color)

    def ypos_monitor_callback(self,msg):
        if msg.y_pos==False:
            self.current_ypos_status= False
            if self.current_ypos_status == False:
                self.update_ypos_status_light("red")
        else:
            self.update_ypos_status_light("green")
        self.get_logger().info(f'Received state {msg.y_pos}\n')
    
    def update_ypos_status_light(self, color):
        self.canvas.itemconfig(self.ypos_light, fill=color)


    def xpos_monitor_callback(self,msg):
        if msg.x_pos==False:
            self.current_xpos_status= False
            if self.current_xpos_status == False:
                self.update_xpos_status_light("red")
        else:
            self.update_xpos_status_light("green")
        self.get_logger().info(f'Received state {msg.x_pos}\n')
    
    def update_xpos_status_light(self, color):
        self.canvas.itemconfig(self.xpos_light, fill=color)

    def rtk_status_monitor_callback(self,msg):

        if msg.rtk_status==False:
            self.current_rtk_status= False
            if self.current_rtk_status == False:
                self.update_rtk_status_light("red")
        else:
            self.update_rtk_status_light("green")
        self.get_logger().info(f'Received state {msg.rtk_status}\n')


    def update_rtk_status_light(self, color):
        self.canvas.itemconfig(self.rtk_light, fill=color)

    def lidar_status_monitor_callback(self,msg):

        if msg.lidar_frequency_validator==False:
            self.current_lidar_status= False
            if self.current_lidar_status== False:
                self.update_lidar_status_light("red")
        else:
            self.update_lidar_status_light("green")
        self.get_logger().info(f'Received state {msg.lidar_frequency_validator}\n')


    def update_lidar_status_light(self, color):
        self.canvas.itemconfig(self.lidar_light, fill=color)

    def localization1_status_monitor_callback(self,msg):

        if msg.gps_position_status_validator==False:
            self.current_localization1_status= False
            if self.current_localization1_status== False:
                self.update_localization1_status_light("red")
        else:
            self.update_localization1_status_light("green")
        self.get_logger().info(f'Received state {msg.gps_position_status_validator}\n')


    def update_localization1_status_light(self, color):
        self.canvas.itemconfig(self.localization_light1, fill=color)

    def localization2_status_monitor_callback(self,msg):

        if msg.gps_position_status_validator2==False:
            self.current_localization2_status= False
            if self.current_localization2_status== False:
                self.update_localization2_status_light("red")
        else:
            self.update_localization2_status_light("green")
        self.get_logger().info(f'Received state {msg.gps_position_status_validator2}\n')


    def update_localization2_status_light(self, color):
        self.canvas.itemconfig(self.localization_light2, fill=color)

    def check_ros(self):
        rclpy.spin_once(self,timeout_sec=0.1)
        self.root.after(100,self.check_ros)

    def on_closing(self):
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()  # Stops the Tkinter main loop

def main(args=None):
    rclpy.init(args=args)
    sensor_reader = SensorReader()
    #rclpy.spin(sensor_reader)

    try:
        sensor_reader.init_ui()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()