
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String  # For publishing sensor state as string
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Temperature, NavSatFix

from geometry_msgs.msg import TwistStamped
import json
from interface.msg import ErrorMsg
import time

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Initialize dictionary to store sensor states
        self.sensor_states = {
            'linear_velocity_sensor': None,
            'temp_sensor': None,
            'velocity_sensor': None,
            'lidar_frequency': None,
            'rtk_status': None
        }
        
        # Frequency monitoring attributes
        self.last_time_msg_published = 0


        # Create subscribers
        self.linear_velocity_subscriber = self.create_subscription(Odometry,'/hus/platform/odom',self.linear_velocity_callback,10)
        self.lidar_frequency_subscriber= self.create_subscription(PointCloud2,'/hus/ouster/points', self.lidar_callback, qos_profile_sensor_data)
        self.temperature_subscriber = self.create_subscription(Temperature,'/hus/imu/temp',self.temp_callback,10)
        self.random_subscriber = self.create_subscription(TwistStamped,'/hus/imu/velocity',self.random_callback,10)
        self.rtk_gps_subscriber = self.create_subscription(NavSatFix,'/hus/imu/nav_sat_fix',self.rtk_status_callback,10)

        self.get_logger().info('Subscription to /hus/imu/nav_sat_fix has been created')
        self.rtk_gps_subscriber # prevent unused variable warning

        
        # Create publisher
        self.states_publisher = self.create_publisher(String, 'state_topic', 10)
        self.alert_publisher = self.create_publisher(ErrorMsg, 'temperature_alert', 10)

        # Create a timer to periodically check and publish state information
        self.timer2 = self.create_timer(10.0, self.check_and_publish)
        self.timer = self.create_timer(1.0,self.publish_alerts)

    # Initialize the time of the last LiDAR message
        self.last_lidar_time = None

    def rtk_status_callback(self,msg):
        self.get_logger().info('Received a message on /hus/imu/nav_sat_fix')
        self.sensor_states['rtk_status'] = msg.status.status
        #self.get_logger().info(f'Current Status: {msg.status.status}')
    def linear_velocity_callback(self, msg):
        self.sensor_states['linear_velocity_sensor'] = msg.twist.twist.linear.x  # Accessing the linear x value

    def temp_callback(self, msg):
        self.sensor_states['temp_sensor'] = msg.temperature  # Accessing the temperature value

    def random_callback(self, msg):
        self.sensor_states['velocity_sensor'] = msg.twist.angular.x # Accessing the angular x value
    def lidar_callback(self, msg):
        current_time_seconds = self.get_clock().now().nanoseconds
        time_in_between_published_msgs = (current_time_seconds)-self.last_time_msg_published
        hz_calculator = (1/time_in_between_published_msgs) / .000000001
        self.get_logger().info(f'Frequency_published {hz_calculator}')
        self.last_time_msg_published = current_time_seconds

    def check_and_publish(self):
        state_msg = String()
        # Convert dictionary to JSON string for publishing
        state_msg.data = json.dumps(self.sensor_states)
        self.states_publisher.publish(state_msg)

    def publish_alerts(self):
        temperature = self.sensor_states.get('temp_sensor')
        x_velocity = self.sensor_states.get('linear_velocity_sensor')
        alert_msg = ErrorMsg()
        
        if temperature is not None:
            alert_msg.current_temperature = temperature
            alert_msg.too_high = temperature > 34.4  # Set threshold for too high
        
        if x_velocity is not None:
            alert_msg.linear_velocity = x_velocity
            alert_msg.too_fast = x_velocity > 0.01
           

        self.alert_publisher.publish(alert_msg)
        #self.get_logger().info(f'ERROR MSG: current TEMP {alert_msg.current_temperature} too high: {alert_msg.too_high}')
        #self.get_logger().info(f'ERROR MSG: {alert_msg.linear_velocity} too fast: {alert_msg.too_fast}')


def main(args=None):
    rclpy.init(args=args)
    
    sensor_node = SensorNode()
    
    rclpy.spin(sensor_node)
    
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
