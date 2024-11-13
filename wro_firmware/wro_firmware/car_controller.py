#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import threading

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')

        # Initialize serial communication
        try:
            self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  # Replace with your serial port
            self.get_logger().info('Serial port opened successfully.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error opening serial port: {e}')
            rclpy.shutdown()
            return

        # Subscribe to /drive and /wheel_steer topics
        self.drive_subscription = self.create_subscription(
            Float64,
            '/drive',
            self.drive_callback,
            10)
        self.steer_subscription = self.create_subscription(
            Float64,
            '/steer_angle',
            self.steer_callback,
            10)

        # Initialize variables
        self.velocity = 0.0
        self.angle = 0.0

        # Lock for thread-safe serial communication
        self.lock = threading.Lock()

    def drive_callback(self, msg):
        self.velocity = msg.data
        self.send_velocity_command()

    def steer_callback(self, msg):
        self.angle = msg.data
        self.send_angle_command()

    def send_velocity_command(self):
        with self.lock:
            # Construct the velocity command string
            command = 'v{:.4f},'.format(self.velocity)
            # Send command over serial
            try:
                self.ser.write(command.encode())
                self.get_logger().debug(f'Sent velocity command: {command}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error writing to serial port: {e}')

    def send_angle_command(self):
        with self.lock:
            # Construct the angle command string
            command = 'a{:.4f},'.format(self.angle)
            # Send command over serial
            try:
                self.ser.write(command.encode())
                self.get_logger().debug(f'Sent angle command: {command}')
            except serial.SerialException as e:
                self.get_logger().error(f'Error writing to serial port: {e}')

    def destroy_node(self):
        # Close the serial port when shutting down
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    car_controller = CarController()

    try:
        rclpy.spin(car_controller)
    except KeyboardInterrupt:
        pass
    finally:
        car_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()