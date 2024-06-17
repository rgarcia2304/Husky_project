
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String  # For publishing sensor state as string
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Temperature, NavSatFix
from sbg_driver.msg import SbgGpsPos, SbgEkfNav
from geometry_msgs.msg import TwistStamped
import json
from interface.msg import ErrorMsg
import time
import psutil
import subprocess

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Initialize dictionary to store sensor states
        self.sensor_states = {
            'lidar_frequency': None,
            'rtk_status': None,
            'gps_pos_status': None,
            'gps_pos_status2': None,
            'x_pos_status': None,
            'y_pos_status': None,
            'z_pos_status': None,
        }
        
        # Frequency monitoring attributes
        self.last_time_msg_published = 0
        # Create subscribers
    
        self.lidar_frequency_subscriber= self.create_subscription(PointCloud2,'/hus/ouster/points', self.lidar_callback, qos_profile_sensor_data)
        self.rtk_gps_subscriber = self.create_subscription(NavSatFix,'/hus/imu/nav_sat_fix',self.rtk_status_callback,10)
        #Both of these are used to validate the positon solution
        self.gps_position_status_subscriber = self.create_subscription(SbgGpsPos,'/hus/sbg/gps_pos',self.gps_postion_status_callback,10)
        self.gps_position_status_subscriber2 = self.create_subscription(SbgEkfNav,'/hus/sbg/ekf_nav',self.gps_postion_status_callback2,10)
        self.x_position_accuracy_subscriber = self.create_subscription(SbgEkfNav,'/hus/sbg/ekf_nav',self.x_postion_status_callback,10)
        self.y_position_accuracy_subscriber = self.create_subscription(SbgEkfNav,'/hus/sbg/ekf_nav',self.y_postion_status_callback,10)
        self.z_position_accuracy_subscriber = self.create_subscription(SbgEkfNav,'/hus/sbg/ekf_nav',self.z_postion_status_callback,10)

        self.rtk_gps_subscriber # prevent unused variable warning
        # Create publisher
        self.states_publisher = self.create_publisher(String, 'state_topic', 10)
        self.alert_publisher = self.create_publisher(ErrorMsg, 'temperature_alert', 10)

        # Create a timer to periodically check and publish state information
        self.timer2 = self.create_timer(10.0, self.check_and_publish)
        self.timer = self.create_timer(1.0,self.publish_alerts)

    # Initialize the time of the last LiDAR message
        self.last_lidar_time = None
        self.rosbag_check_timer = self.create_timer(5.0, self.check_rosbag_recording)

    def x_postion_status_callback(self, msg):
        self.sensor_states['x_pos_status'] = msg.position_accuracy.x
        #self.get_logger().info(f'Frequency_published {msg.position_accuracy.x}')
    def y_postion_status_callback(self, msg):
        self.sensor_states['y_pos_status'] = msg.position_accuracy.y

    def z_postion_status_callback(self, msg):
        self.sensor_states['z_pos_status'] = msg.position_accuracy.z

    def gps_postion_status_callback(self, msg):
        self.sensor_states['gps_pos_status'] = msg.status.type
        #self.get_logger().info(f'Frequency_published {msg.status.type}')

    def gps_postion_status_callback2(self, msg):
        self.sensor_states['gps_pos_status2'] = msg.status.solution_mode
        #self.get_logger().info(f'Frequency_published {msg.status.solution_mode}')

    def rtk_status_callback(self,msg):
        #self.get_logger().info('Received a message on /hus/imu/nav_sat_fix')
        self.sensor_states['rtk_status'] = msg.status.status
        #self.get_logger().info(f'Current Status: {msg.status.status}')

    def lidar_callback(self, msg):
        current_time_seconds = self.get_clock().now().nanoseconds
        time_in_between_published_msgs = (current_time_seconds)-self.last_time_msg_published
        hz_calculator = (1/time_in_between_published_msgs) / .000000001
        self.sensor_states['lidar_frequency'] = hz_calculator
        self.last_time_msg_published = current_time_seconds

    def check_rosbag_recording(self):
        if self.is_rosbag_recording():
            #self.get_logger().info("Rosbag is recording")
            self.sensor_states['rosbag_recording'] = True
        else:
            #self.get_logger().info("Rosbag is not recording")
            self.sensor_states['rosbag_recording'] = False

    def is_rosbag_recording(self):
        """Check if ros2 bag record is running."""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            if 'ros2' in proc.info['cmdline'] and 'bag' in proc.info['cmdline'] and 'record' in proc.info['cmdline']:
                return True
        return False

    def check_and_publish(self):
        state_msg = String()
        # Convert dictionary to JSON string for publishing
        state_msg.data = json.dumps(self.sensor_states)
        self.states_publisher.publish(state_msg)

    def publish_alerts(self):
        rtk_status_check = self.sensor_states.get('rtk_status')
        gps_pos_status_check = self.sensor_states.get('gps_pos_status')
        gps_pos_status_check2 = self.sensor_states.get('gps_pos_status2')
        lidar_status_check = self.sensor_states.get('lidar_frequency')
        x_pos_status_check = self.sensor_states.get('x_pos_status')
        y_pos_status_check = self.sensor_states.get('y_pos_status')
        z_pos_status_check = self.sensor_states.get('z_pos_status')
        alert_msg = ErrorMsg()
        
        if rtk_status_check is not None:
            #self.get_logger().info(f'status number {rtk_status_check}')
            if rtk_status_check !=0:

                alert_msg.rtk_status = False
                #self.get_logger().info(f'status number {alert_msg.rtk_status}')
            else:
                
                alert_msg.rtk_status= True
                #self.get_logger().info(f'status number {alert_msg.rtk_status}')
        
        if lidar_status_check is not None:
            if lidar_status_check<8:
                alert_msg.lidar_frequency_validator = False
            else:
                alert_msg.lidar_frequency_validator = True
        #checks for localizationn solutjion
        if gps_pos_status_check >=7 :
            alert_msg.gps_position_status_validator= True
        else:
            alert_msg.gps_position_status_validator= False

        if gps_pos_status_check2 ==4:
            alert_msg.gps_position_status_validator2= True
        else:
            alert_msg.gps_position_status_validator2= False

        if abs(x_pos_status_check-.03)>0.005:
            alert_msg.x_pos =False
        else:
            alert_msg.x_pos = True

        if abs(y_pos_status_check-.03)>0.025:
            alert_msg.y_pos =False
        else:
            alert_msg.y_pos = True

        if abs(y_pos_status_check-.03)>0.025:
            alert_msg.y_pos =False
        else:
            alert_msg.y_pos = True
        
        if abs(y_pos_status_check-.03)>0.025:
            alert_msg.z_pos =False
        else:
            alert_msg.z_pos = True


        self.get_logger().info(f'STATUS {y_pos_status_check }')
        self.get_logger().info(f'STATUS {alert_msg.y_pos }')

        self.alert_publisher.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    
    sensor_node = SensorNode()
    
    rclpy.spin(sensor_node)
    
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
