
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String  # For publishing sensor state as string
from sensor_msgs.msg import PointCloud2, NavSatFix, BatteryState
from sbg_driver.msg import SbgGpsPos, SbgEkfNav
import json
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from interface.msg import HuskyStateStatus
import time
import psutil
import subprocess



#reads 
class huskyStatePublisher(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Initialize dictionary to store sensor states
        self.sensor_states = {
            'lidar_frequency': None,
            'rtk_status': None,
            'status_type': None,
            'solution_mode_status': None,
            'position_status':None,
            'rosbag_recording':None,
            'battery_status':None,
        }
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        # Frequency monitoring attributes
        self.last_time_msg_published = 0
        # Create subscribers
        self.lidar_frequency_subscriber= self.create_subscription(PointCloud2,'/hus/ouster/points', self.lidar_callback, qos_profile_sensor_data)
        self.rtk_gps_subscriber = self.create_subscription(NavSatFix,'/hus/imu/nav_sat_fix',self.rtk_status_callback,10)
        self.status_type_subscriber = self.create_subscription(SbgGpsPos,'/hus/sbg/gps_pos',self.status_type_callback,10)
        self.solution_status_subscriber = self.create_subscription(SbgEkfNav,'/hus/sbg/ekf_nav',self.solution_mode_status_callback,10)
        self.x_position_accuracy_subscriber = self.create_subscription(SbgEkfNav,'/hus/sbg/ekf_nav',self.postion_status_callback,10)
        self.battery_voltage_subscriber = self.create_subscription(BatteryState,'/hus/platform/bms/state',self.battery_status_callback,qos_profile)
        self.rtk_gps_subscriber # prevent unused variable warning
        # Create publisher for all data json dictionary representation
        self.states_publisher = self.create_publisher(String, 'state_topic', 10)
        #Creates publishers that updates all are state messages 
        self.alert_publisher = self.create_publisher(HuskyStateStatus, 'status_alert', 10)
        # Create a timer to periodically check and publish state information
        self.timer2 = self.create_timer(1, self.check_and_publish)
        self.timer = self.create_timer(.2,self.publish_alerts)
        
        # Initialize the time of the last LiDAR message
        self.last_lidar_time = None
        self.rosbag_check_timer = self.create_timer(.1, self.check_rosbag_recording)

    def battery_status_callback(self, msg):
        self.sensor_states['battery_status'] = msg.voltage

    def postion_status_callback(self, msg):
        position_calculator = (msg.position_accuracy.x)*(msg.position_accuracy.z) * (msg.position_accuracy.y)
        self.sensor_states['position_status'] = position_calculator

    def status_type_callback(self, msg):
        self.sensor_states['status_type'] = msg.status.type
        #self.get_logger().info(f'Frequency_published {msg.status.type}')

    def solution_mode_status_callback(self, msg):
        self.sensor_states['solution_mode_status'] = msg.status.solution_mode
        #self.get_logger().info(f'Frequency_published {msg.status.solution_mode}')

    def rtk_status_callback(self,msg):
        #self.get_logger().info('Received a message on /hus/imu/nav_sat_fix')
        self.sensor_states['rtk_status'] = msg.status.status
        #self.get_logger().info(f'Current Status: {msg.status.status}')

    def lidar_callback(self, msg):
        current_time_seconds = self.get_clock().now().nanoseconds
        #self.get_logger().info(f'Current Status: {current_time_seconds}')
        time_in_between_published_msgs = (current_time_seconds)-self.last_time_msg_published
        #self.get_logger().info(f'Current Status: {time_in_between_published_msgs}')
        hz_calculator = (1/time_in_between_published_msgs) / .000000001
        #self.get_logger().info(f'Hello: {hz_calculator}')
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
        self.get_logger().info(f'Current Status: {state_msg.data}')
        self.states_publisher.publish(state_msg)

    def publish_alerts(self):       
        alert_msg = HuskyStateStatus()

        #conditonals for determining whether states are valid or not
        #conditonal for the rtk_status
        if self.sensor_states.get('rtk_status') !=0:
            alert_msg.rtk_status = False
        else:
            alert_msg.rtk_status= True

        #check to see if lidar frequency is publishing at desired frequency
        if self.sensor_states.get('lidar_frequency') is not None:
            if self.sensor_states.get('lidar_frequency')<1:
                alert_msg.lidar_frequency_validator = False
            else:
                alert_msg.lidar_frequency_validator = True

        #checks to see if status type message is valid
        if self.sensor_states.get('status_type') != None:
            if self.sensor_states.get('status_type') >=7 :
                alert_msg.status_type_validator= True
            else:
                alert_msg.status_type_validator = False

        #checks to see if solution mode status is valid
        if self.sensor_states.get('solution_mode_status') ==4:
            alert_msg.solution_mode_validator= True
        else:
            alert_msg.solution_mode_validator= False

        #checks to see if position status is valid 
        if self.sensor_states.get('position_status') > 0.000002:
            alert_msg.position_status = False
        else:
            alert_msg.position_status = True

        if self.sensor_states.get('rosbag_recording') == False:
            alert_msg.is_recording = False
        else:
            alert_msg.is_recording = True
        
        if self.sensor_states.get('battery_status') != None:

            if self.sensor_states.get('battery_status') < 24:
                alert_msg.battery_isok = False
            else:
                alert_msg.battery_isok = True

        self.alert_publisher.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_node = huskyStatePublisher()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()