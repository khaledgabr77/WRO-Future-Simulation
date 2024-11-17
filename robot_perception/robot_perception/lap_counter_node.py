import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray

class LapCounterNode(Node):
    def __init__(self):
        super().__init__('lap_counter_node')
        self.lap_count = 0
        self.blue_line_visible = False
        self.image_height = 480  
        self.bottom_threshold = 50  # Threshold in pixels from the bottom
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10)
        self.subscription  

        # Set the logger level to DEBUG to see all messages
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    def detection_callback(self, msg):
        blue_line_at_bottom_detected = False

        self.get_logger().debug('Processing new detection array')
        for detection in msg.detections:
            if self.is_blue_line(detection):
                self.get_logger().debug(f'Detected blue line with bbox center at y={detection.bbox.center.position.y}')
                if self.is_at_bottom(detection):
                    self.get_logger().debug('Blue line is at the bottom of the image')
                    blue_line_at_bottom_detected = True
                    break  # Blue line detected at the bottom, no need to check further
                else:
                    self.get_logger().debug('Blue line is not at the bottom of the image')

        # Check for transition from visible to not visible
        if self.blue_line_visible and not blue_line_at_bottom_detected:
            self.lap_count += 1
            self.get_logger().info(f'Lap count incremented to: {self.lap_count}')
            self.blue_line_visible = False
        elif blue_line_at_bottom_detected:
            if not self.blue_line_visible:
                self.get_logger().debug('Blue line has appeared at the bottom')
            self.blue_line_visible = True
        else:
            if self.blue_line_visible:
                self.get_logger().debug('Blue line is no longer at the bottom')
                self.blue_line_visible = False

    def is_blue_line(self, detection):
        # Check if the detection is the blue line
        result = detection.class_name == "blue_line"
        self.get_logger().debug(f'Checking if detection is blue line: {result}')
        return result

    def is_at_bottom(self, detection):
        # Check if the blue line is at the bottom of the image
        bbox = detection.bbox
        bbox_bottom = bbox.center.position.y + bbox.size.y / 2
        is_bottom = bbox_bottom >= self.image_height - self.bottom_threshold
        self.get_logger().debug(f'Checking if blue line is at bottom: bbox_bottom={bbox_bottom}, is_bottom={is_bottom}')
        return is_bottom

def main(args=None):
    rclpy.init(args=args)
    lap_counter = LapCounterNode()
    
    try:
        rclpy.spin(lap_counter)
    except KeyboardInterrupt:
        pass
    finally:
        lap_counter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
