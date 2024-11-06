import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

class ArduinoCommunicator(Node):
    def _init_(self):
        super()._init_('arduino_communicator')

        # Declare parameters with default values
        self.declare_parameter('steer_min', -3.14)  # Minimum steering angle in radians
        self.declare_parameter('steer_max', 3.14)   # Maximum steering angle in radians
        self.declare_parameter('drive_min', 0.0)     # Minimum motor speed in rad/s
        self.declare_parameter('drive_max', 10.0)    # Maximum motor speed in rad/s
        
        # Initialize serial communication
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)  # Update with the correct port
        self.create_subscription(Float64, '/steer', self.steer_callback, 10)
        self.create_subscription(Float64, '/drive', self.drive_callback, 10)

    def steer_callback(self, msg):
        steer_min = self.get_parameter('steer_min').value
        steer_max = self.get_parameter('steer_max').value
        limited_value = self.limit_value(msg.data, steer_min, steer_max)
        self.send_to_arduino('S', limited_value)

    def drive_callback(self, msg):
        drive_min = self.get_parameter('drive_min').value
        drive_max = self.get_parameter('drive_max').value
        limited_value = self.limit_value(msg.data, drive_min, drive_max)
        self.send_to_arduino('D', limited_value)

    def send_to_arduino(self, command, value):
        # Send command character followed by the value
        self.serial_port.write(f'{command}:{value:.2f}\n'.encode('utf-8'))

    def limit_value(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

def main(args=None):
    rclpy.init(args=args)
    communicator = ArduinoCommunicator()
    rclpy.spin(communicator)
    communicator.serial_port.close()
    rclpy.shutdown()

if _name_ == '_main_':
    main()