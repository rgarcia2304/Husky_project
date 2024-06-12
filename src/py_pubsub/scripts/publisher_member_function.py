# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        #Odemetry is the message tyye, second is the topic name, the third is the callback and the fourth slot is the queue
        self.subscriber = self.create_subscription(Odometry, '/hus/platform/odom',self.listener_callback,10)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscriber
        self.publisher_
        

    def listener_callback(self, msg):
        print('condition not met')
        ''''''
        if msg.twist.twist.linear.x > 0.01:
            self.get_logger().info('Linear x is greater than 0.01: %f' % msg.twist.twist.linear.x)
            notification_msg = String()
            notification_msg.data = 'Condition met'
            self.publisher_.publish(notification_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
