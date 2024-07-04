import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String  # For publishing sensor state as string
from sensor_msgs.msg import PointCloud2, NavSatFix, BatteryState
from sbg_driver.msg import SbgGpsPos, SbgEkfNav
import json
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from husky_monitored_messages.msg import HuskyStateStatus
import time
import psutil
import subprocess

# Class for publishing the state of the Husky robot's sensors
class HuskyStatePublisher(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Initialize dictionary to store sensor states
        self.sensor_states = {
            'lidar_frequency': None,
            'rtk_status': None,
            'status_type': None,
            'solution_mode_status': None,
            'position_status': None,
            'rosbag_recording': None,
            'battery_status': None,
        }

        # Quality of Service (QoS) profile settings for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Initialize frequency monitoring attribute
        self.last_time_msg_published = 0
        
        # Create subscribers for various sensor topics
        self.lidar_frequency_subscriber = self.create_subscription(
            PointCloud2, '/hus/ouster/points', self.lidar_callback, qos_profile_sensor_data)
        self.rtk_gps_subscriber = self.create_subscription(
            NavSatFix, '/hus/imu/nav_sat_fix', self.rtk_status_callback, 10)
        self.status_type_subscriber = self.create_subscription(
            SbgGpsPos, '/hus/sbg/gps_pos', self.status_type_callback, 10)
        self.solution_status_subscriber = self.create_subscription(
            SbgEkfNav, '/hus/sbg/ekf_nav', self.solution_mode_status_callback, 10)
        self.x_position_accuracy_subscriber = self.create_subscription(
            SbgEkfNav, '/hus/sbg/ekf_nav', self.postion_status_callback, 10)
        self.battery_voltage_subscriber = self.create_subscription(
            BatteryState, '/hus/platform/bms/state', self.battery_status_callback, qos_profile)

        # Create publishers for sensor states and alerts
        self.states_publisher = self.create_publisher(String, 'state_topic', 10)
        self.alert_publisher = self.create_publisher(HuskyStateStatus, 'status_alert', 10)
        
        # Create timers for periodic state checks and publishing
        self.timer2 = self.create_timer(1, self.check_and_publish)
        self.timer = self.create_timer(0.2, self.publish_alerts)
        
        # Initialize the time of the last LiDAR message
        self.last_lidar_time = None
        
        # Timer to periodically check if rosbag is recording
        self.rosbag_check_timer = self.create_timer(0.1, self.check_rosbag_recording)

    def battery_status_callback(self, msg):
        """Callback to update battery status from the BatteryState message."""
        self.sensor_states['battery_status'] = msg.voltage

    def postion_status_callback(self, msg):
        """Callback to update position status from the SbgEkfNav message."""
        position_calculator = msg.position_accuracy.x * msg.position_accuracy.z * msg.position_accuracy.y
        self.sensor_states['position_status'] = position_calculator

    def status_type_callback(self, msg):
        """Callback to update status type from the SbgGpsPos message."""
        self.sensor_states['status_type'] = msg.status.type

    def solution_mode_status_callback(self, msg):
        """Callback to update solution mode status from the SbgEkfNav message."""
        self.sensor_states['solution_mode_status'] = msg.status.solution_mode

    def rtk_status_callback(self, msg):
        """Callback to update RTK GPS status from the NavSatFix message."""
        self.sensor_states['rtk_status'] = msg.status.status

    def lidar_callback(self, msg):
        """Callback to update LiDAR frequency from the PointCloud2 message."""
        current_time_seconds = self.get_clock().now().nanoseconds
        time_in_between_published_msgs = current_time_seconds - self.last_time_msg_published
        hz_calculator = (1 / time_in_between_published_msgs) / 0.000000001
        self.sensor_states['lidar_frequency'] = hz_calculator
        self.last_time_msg_published = current_time_seconds

    def check_rosbag_recording(self):
        """Check if ros2 bag record is running and update state."""
        self.sensor_states['rosbag_recording'] = self.is_rosbag_recording()

    def is_rosbag_recording(self):
        """Determine if ros2 bag record process is running."""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            if 'ros2' in proc.info['cmdline'] and 'bag' in proc.info['cmdline'] and 'record' in proc.info['cmdline']:
                return True
        return False

    def check_and_publish(self):
        """Periodically publish the current sensor states as a JSON string."""
        state_msg = String()
        state_msg.data = json.dumps(self.sensor_states)
        self.get_logger().info(f'Current Status: {state_msg.data}')
        self.states_publisher.publish(state_msg)

    def publish_alerts(self):
        """Publish alerts based on the sensor states."""
        alert_msg = HuskyStateStatus()

        # Conditional checks for sensor validity and alert creation
        alert_msg.rtk_status = self.sensor_states.get('rtk_status') == 0

        if self.sensor_states.get('lidar_frequency') is not None:
            alert_msg.lidar_frequency_validator = self.sensor_states.get('lidar_frequency') >= 1

        if self.sensor_states.get('status_type') is not None:
            alert_msg.status_type_validator = self.sensor_states.get('status_type') >= 7

        alert_msg.solution_mode_validator = self.sensor_states.get('solution_mode_status') == 4

        if self.sensor_states.get('position_status') is not None:
            alert_msg.position_status = self.sensor_states.get('position_status') <= 0.00002

        alert_msg.is_recording = self.sensor_states.get('rosbag_recording', False)

        if self.sensor_states.get('battery_status') is not None:
            alert_msg.battery_isok = self.sensor_states.get('battery_status') >= 24.2
            battery_status= self.sensor_states.get('battery_status')
            self.get_logger().info(f'Current Status: {battery_status}')

        self.alert_publisher.publish(alert_msg)

def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    sensor_node = HuskyStatePublisher()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
