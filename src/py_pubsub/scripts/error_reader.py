
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
        self.current_rtk_status = None
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

        #rtk display
        self.rtk_light= self.canvas.create_oval(350, 250, 450, 350, fill="yellow")
        self.canvas.create_text(485, 485, text="Monitoring: rtk_status Alert", anchor=tk.CENTER)

        #closing actions
        self.root.after(100, self.check_ros)
        self.root.mainloop()
        

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
