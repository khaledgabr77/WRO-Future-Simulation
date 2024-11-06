#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import serial
import threading

class CarController:
    def __init__(self):
        # Initialize serial communication
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # Replace with your serial port

        # Initialize ROS node
        rospy.init_node('car_controller', anonymous=True)

        # Subscribe to /drive and /wheel_steer topics
        rospy.Subscriber('/drive', Float64, self.drive_callback)
        rospy.Subscriber('/wheel_steer', Float64, self.steer_callback)

        # Initialize variables
        self.velocity = 0.0
        self.angle = 0.0

        # Lock for thread-safe serial communication
        self.lock = threading.Lock()

    def drive_callback(self, msg):
        self.velocity = msg.data
        self.send_command()

    def steer_callback(self, msg):
        self.angle = msg.data
        self.send_command()

    def send_command(self):
        with self.lock:
            # Construct the command string
            command = 'v{:.4f},a{:.4f},'.format(self.velocity, self.angle)
            # Send command over serial
            self.ser.write(command.encode())

    def run(self):
        rospy.spin()
        self.ser.close()

if __name__ == '__main__':
    try:
        controller = CarController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
