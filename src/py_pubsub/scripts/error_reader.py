
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
        self.status_callback = self.create_subscription(ErrorMsg,'temperature_alert',self.status_callback,10)

        self.current_zpos_status = None
        self.current_xpos_status = None
        self.current_ypos_status = None
        self.current_rtk_status = None
        self.current_lidar_status = None
        self.current_localization1_status = None
        self.current_localization2_status = None
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

    def status_callback(self,msg):
        self.current_zpos_status = msg.z_pos
        self.current_xpos_status = msg.x_pos
        self.current_ypos_status = msg.z_pos
        self.current_rtk_status = msg.rtk_status
        self.current_lidar_status = msg.lidar_frequency_validator
        self.current_localization1_status = msg.gps_position_status_validator
        self.current_localization2_status = msg.gps_position_status_validator2
        #light updator based on status
        self.update_light(self.rtk_light,self.current_rtk_status)
        self.update_light(self.lidar_light,self.current_lidar_status)
        self.update_light(self.localization_light1,self.current_xpos_status)
        self.update_light(self.localization_light2,self.current_ypos_status)
        self.update_light(self.xpos_light,self.current_zpos_status)
        self.update_light(self.ypos_light,self.current_localization1_status)
        self.update_light(self.zpos_light,self.current_localization2_status)

    def update_light(self, light, status):
        color = 'green' if status else 'red'
        if self.canvas is not None:
            self.canvas.itemconfig(light, fill=color)
    
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