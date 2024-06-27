
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from interface.msg import HuskyStateStatus
import tkinter as tk



class digitalStateDisplay(Node):
    def __init__(self):
        super().__init__('error_reader')        
        # Create subscribers
        self.status_callback = self.create_subscription(HuskyStateStatus,'status_alert',self.status_callback,10)
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
        #light positions on GUI display
        self.rtk_light= self.canvas.create_oval(30, 30, 40, 40, fill="yellow")
        self.lidar_light= self.canvas.create_oval(60, 30, 70, 40, fill="yellow")
        self.status_type_light= self.canvas.create_oval(90, 30, 100, 40, fill="yellow")
        self.solution_mode_light= self.canvas.create_oval(120, 30, 130, 40, fill="yellow")
        self.pos_light= self.canvas.create_oval(150, 30, 160, 40, fill="yellow")
        self.battery_light= self.canvas.create_oval(180, 30, 190, 40, fill="yellow")
        #closing actions
        self.root.after(100, self.check_ros)
        self.root.mainloop()

    def status_callback(self,msg):
        #light updator based on current status
        self.update_light(self.rtk_light,msg.rtk_status)
        self.update_light(self.lidar_light,msg.lidar_frequency_validator)
        self.update_light(self.status_type_light,msg.status_type_validator)
        self.update_light(self.solution_mode_light,msg.solution_mode_validator)
        self.update_light(self.pos_light,msg.position_status)
        self.update_light(self.battery_light,msg.battery_isok)

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
    sensor_reader = digitalStateDisplay()
    try:
        sensor_reader.init_ui()
    except KeyboardInterrupt:
        pass
    finally:
        sensor_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()