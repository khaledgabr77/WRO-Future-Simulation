import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class RGBToBGRNode(Node):
    def __init__(self):
        super().__init__('rgb_to_bgr_converter')
        self.subscription = self.create_subscription(
            Image,
            '/input_image',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/output_image',
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        if msg.encoding == 'rgb8':
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            # Convert RGB to BGR
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            # Convert back to ROS Image message
            bgr_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
            bgr_msg.header = msg.header  # Preserve the original header
            # Publish the converted image
            self.publisher.publish(bgr_msg)
        else:
            self.get_logger().info('Image encoding is not rgb8, skipping conversion.')

def main(args=None):
    rclpy.init(args=args)
    node = RGBToBGRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
