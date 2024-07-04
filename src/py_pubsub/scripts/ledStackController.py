import os
import sys

# Get the directory of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the current directory to the Python path
sys.path.append(current_dir)

# Now you can import the module
import time
import rclpy
from rclpy.node import Node
from husky_monitored_messages.msg import HuskyStateStatus
import relayControlsAPI

class LedStackController(Node):


    def __init__(self):
        super().__init__('hardware_node')
        self.status_subscription = self.create_subscription(
            HuskyStateStatus,
            'status_alert',
            self.status_callback,
            10)
        self.physical_hardware = relayControlsAPI.relay_controls()
        self.light_behavior = "none"
        self.led_stack_status = {"red": 0.0, "orange": 0.0, "green": 0.0}
        self.sensor_status = HuskyStateStatus()
        self.prev_cmd = {"red": False, "orange": False, "green": False}
        self.led_stack_decoder = {"red": 1, "orange": 2, "green": 4}
        self.current_state = 0
        self.active_light = None  # Track the currently active light
        self.blink_timer = None  # Added for blinking functionality
        self.blink_interval = 0.5  # Default blink interval
        self.create_timer(0.25, self.light_runner)  # Main loop at 4 Hz
        self.get_logger().info('LedStackController initialized')

    def status_callback(self, msg):
        self.sensor_status = msg
        #self.get_logger().info('Status callback triggered')

    def set_led_stack_from_sensor_state(self):
        self.get_logger().info('set_led_stack_from_sensor_state called')
        if self.sensor_status.lidar_frequency_validator == False:
            self.led_stack_status["red"] = 2.0
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["green"] = 0.0
            self.active_light = "red"

        elif self.sensor_status.battery_isok == False:
            self.led_stack_status["red"] = 2.0
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["green"] = 0.0
            self.active_light = "red"
        elif self.sensor_status.solution_mode_validator == False:
            self.led_stack_status["red"] = 2.0
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["green"] = 0.0
            self.active_light = "red"
        elif self.sensor_status.position_status == False:
            self.led_stack_status["red"] = 0.1
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["green"] = 0.0
            self.active_light = "red"
        elif self.sensor_status.status_type_validator == False:
            self.led_stack_status["red"] = 2.0
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["green"] = 0.0
            self.active_light = "red"
        elif self.sensor_status.battery_isok == False:
            self.led_stack_status["red"] = 2.0
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["green"] = 0.0
            self.active_light = "red"
        elif self.sensor_status.is_recording == False:
            self.led_stack_status["green"] = 0.5
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["red"] = 0.0
            self.active_light = "green"
        else:
            self.led_stack_status["green"] = 2.0
            self.led_stack_status["orange"] = 0.0
            self.led_stack_status["red"] = 0.0
            self.active_light = "green"

    def control_led_from_status(self):
        self.get_logger().info('control_led_from_status called')
        for color in self.led_stack_status:
            print(color)
            print(self.led_stack_status)
            # turns color from on to off
            if self.led_stack_status[color] < 0.1 and self.prev_cmd[color] == True:
                self.current_state = self.led_stack_decoder[color]
                self.physical_hardware.set_relay_state(self.current_state)
                self.prev_cmd[color] = False
                # Stop blink timer if it exists
                if self.blink_timer is not None:
                    self.blink_timer.cancel()
                    self.blink_timer = None
            # turns color from off to on
            elif self.led_stack_status[color] > 0.9 and self.prev_cmd[color] == False:
                self.current_state = self.led_stack_decoder[color]
                self.physical_hardware.set_relay_state(self.current_state)
                self.prev_cmd[color] = True
                # Stop blink timer if it exists
                if self.blink_timer is not None:
                    self.blink_timer.cancel()
                    self.blink_timer = None
            # blinks
            elif 0.1 <= self.led_stack_status[color] <= 0.9:
                self.current_state = self.led_stack_decoder[color] if self.current_state == 0 else 0
                self.physical_hardware.set_relay_state(self.current_state)
                self.prev_cmd[color] = not self.prev_cmd[color]
                self.blink_interval = self.led_stack_status[color]
                # Start blink timer if not already started
                if self.blink_timer is None:
                    self.start_blink_timer()

    def start_blink_timer(self):
        self.blink_timer = self.create_timer(self.blink_interval, self.toggle_active_light)

    def stop_blink_timer(self):
        if self.blink_timer is not None:
            self.blink_timer.cancel()
            self.blink_timer = None

    def toggle_active_light(self):
        self.get_logger().info(f'toggle_active_light called')
        self.current_state = self.led_stack_decoder[self.active_light] if self.current_state == 0 else 0
        self.physical_hardware.set_relay_state(self.current_state)

    def light_runner(self):
        self.get_logger().info('light_runner called')
        self.set_led_stack_from_sensor_state()
        self.control_led_from_status()

def main(args=None):
    rclpy.init(args=args)
    hardware_sensor_node = LedStackController()
    hardware_sensor_node.physical_hardware.open_ports('/dev/ttyACM0', 9600, 1)
    hardware_sensor_node.physical_hardware.get_sw_version()
    hardware_sensor_node.physical_hardware.set_all_relays(True)
    rclpy.spin(hardware_sensor_node)  # Spin the node to keep it alive and processing callbacks
    hardware_sensor_node.destroy_node()
    hardware_sensor_node.physical_hardware.close_ports()
    hardware_sensor_node.physical_hardware.set_all_relays(False)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

