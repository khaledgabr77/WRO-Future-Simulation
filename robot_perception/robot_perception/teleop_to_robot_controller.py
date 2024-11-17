import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TeleopToRobotController(Node):
    def __init__(self):
        super().__init__('teleop_to_robot_controller')

        # Subscribes to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for /drive and /steer_angle topics
        self.drive_publisher = self.create_publisher(Float64, '/drive', 10)
        self.steer_publisher = self.create_publisher(Float64, '/steer_angle', 10)

        self.get_logger().info('TeleopToRobotController node has started.')

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity from Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Publish to /drive
        drive_msg = Float64()
        drive_msg.data = linear_velocity
        self.drive_publisher.publish(drive_msg)

        # Publish to /steer_angle
        steer_msg = Float64()
        steer_msg.data = angular_velocity
        self.steer_publisher.publish(steer_msg)

        self.get_logger().info(f"Published drive: {linear_velocity}, steer_angle: {angular_velocity}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopToRobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
