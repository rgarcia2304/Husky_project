
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
        self.temp_error_reader = self.create_subscription(ErrorMsg, 'temperature_alert',self.temperature_monitor_callback,10 )
        self.velocity_error_reader = self.create_subscription(ErrorMsg, 'temperature_alert',self.linear_velocity_monitor_callback,10 )
        self.current_temp_state = None
        self.current_velocity_state = None
        self.init_ui()

    def monitor_callback(self, msg):
        try:
            # Convert the JSON string back to a dictionary
            state_dict = json.loads(msg.data)
            self.get_logger().info(f'Received state dictionary: {state_dict}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON string: {e}')

    #display information 
    def init_ui(self):
        self.root = tk.Tk()
        self.root.title("Error Display")
        self.canvas = tk.Canvas(self.root, width=500, height=500)
        self.canvas.pack()
        #Temperature light Montior
        self.temp_light = self.canvas.create_oval(50, 50, 150, 150, fill="yellow")
        self.canvas.create_text(100, 180, text="Monitoring: Temperature Alert", anchor=tk.CENTER)
        
        #Velocity Light Monitor
    
        self.linear_velocity_light= self.canvas.create_oval(350, 50, 450, 150, fill="yellow")
        self.canvas.create_text(350, 180, text="Monitoring: Velocity Alert", anchor=tk.CENTER)
    
        
        #closing actions
        self.root.after(100, self.check_ros)
        self.root.mainloop()
        
    def temperature_monitor_callback(self,msg):
        if msg.too_high ==True:
            self.current_temp_state = True
            if self.current_temp_state == True: 
                self.update_temp_light("red")
        else:
            self.update_temp_light("green")

        self.get_logger().info(f'\nReceived temp state {msg.too_high}')

    def linear_velocity_monitor_callback(self,msg):
        if msg.too_fast ==True:
            self.current_velocity_state = True
            if self.current_velocity_state == True:
                self.update_linear_velocity_light("red")
        else:
            self.update_linear_velocity_light("green")
        self.get_logger().info(f'Received state {msg.too_fast}\n')

    def update_temp_light(self, color):
        self.canvas.itemconfig(self.temp_light, fill=color)

    def update_linear_velocity_light(self, color):
        self.canvas.itemconfig(self.linear_velocity_light, fill=color)

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
