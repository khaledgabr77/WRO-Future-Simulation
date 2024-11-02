import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from cv_bridge import CvBridge
from yolov8_msgs.msg import DetectionArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np

from geometry_msgs.msg import PoseArray, Pose
import tf_transformations
import transforms3d


class Lidar2ImageNode(Node):

    def __init__(self):
        super().__init__('lidar2image_node')

        self.cv_bridge_ = CvBridge()

        # Camera intrinsics
        self.camera_info_ = None

        # TF buffer and listener
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # Subscribers using message_filters
        self.image_sub_ = Subscriber(self, Image, "bgr_image")
        self.detections_sub_ = Subscriber(self, DetectionArray, "/yolo/detections")
        self.scan_sub_ = Subscriber(self, LaserScan, "/scan")

        # Synchronizer
        self.detection_image_scan_sync = ApproximateTimeSynchronizer(
            [self.detections_sub_, self.image_sub_, self.scan_sub_], queue_size=100, slop=0.1)
        self.detection_image_scan_sync.registerCallback(self.detection_image_scan_callback)

        # Camera info subscriber
        self.caminfo_sub_ = self.create_subscription(
            CameraInfo, 'camera_info', self.caminfoCallback, 10)

        # Publisher for object positions
        self.object_positions_pub_ = self.create_publisher(PoseArray, 'detected_object_positions', 10)

        # Publisher for overlay image
        self.overlay_image_pub_ = self.create_publisher(Image, 'overlay_image', 10)

        # Publisher for labeled laser scan
        self.labeled_scan_pub_ = self.create_publisher(LaserScan, 'labeled_scan', 10)
        self.get_logger().info("Initialized lidar2image_node")

    def detection_image_scan_callback(self, detections_msg, image_msg, scan_msg):
        """
        Callback for synchronized detections, RGB images, and laser scans.
        """
        self.get_logger().info("Processing synchronized messages")

        # Ensure camera_info_ is available
        if self.camera_info_ is None:
            self.get_logger().warn("Camera info not yet received. Cannot process data.")
            return
        # self.get_logger().warn(f'intensities : {scan_msg.intensities}')

        # Get camera intrinsics
        fx = self.camera_info_['fx']
        fy = self.camera_info_['fy']
        cx = self.camera_info_['cx']
        cy = self.camera_info_['cy']

        # Get laser scan data and compute LiDAR points
        points_lidar, indices = self.compute_lidar_points(scan_msg)

        # Transform LiDAR points to camera frame
        time_stamp = rclpy.time.Time.from_msg(scan_msg.header.stamp)
        points_camera = self.transform_lidar_points_to_camera_frame(
            points_lidar, scan_msg.header.frame_id, image_msg.header.frame_id, time_stamp)

        if points_camera is None:
            # Transformation failed
            self.get_logger().warn("Could not transform lidar points to camera")
            return

        # Project points onto image plane
        u, v, x_cam, y_cam, z_cam, indices = self.project_points_to_image_plane(
            points_camera, fx, fy, cx, cy, image_msg, indices)

        # Initialize labels array
        labels = np.zeros_like(indices, dtype=int)

        # Associate projected points with detections and estimate object positions
        pose_array_msg, labels = self.associate_points_with_detections(
            u, v, x_cam, y_cam, z_cam, indices, labels, detections_msg, image_msg)
            

        # Publish the object positions
        self.object_positions_pub_.publish(pose_array_msg)

        # Convert image_msg to CV image
        cv_image = self.cv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Draw all projected LiDAR points (before labeling)
        # for i in range(len(u)):
        #     cv2.circle(cv_image, (u[i], v[i]), 2, (100, 100, 100), -1)  # Blue points

        # Draw bounding boxes and labels for red and green objects
        for detection in detections_msg.detections:
            class_id = detection.class_id

            # Process only red and green objects (assuming class IDs 0 and 1)
            if class_id in [1, 2]:
                bbox = detection.bbox
                image_width = image_msg.width
                image_height = image_msg.height

                # Bounding box coordinates
                xmin = int(bbox.center.position.x - bbox.size.x / 2)
                ymin = int(bbox.center.position.y - bbox.size.y / 2)
                xmax = int(bbox.center.position.x + bbox.size.x / 2)
                ymax = int(bbox.center.position.y + bbox.size.y / 2)

                # Clip coordinates to image dimensions
                xmin = max(0, xmin)
                ymin = max(0, ymin)
                xmax = min(image_width - 1, xmax)
                ymax = min(image_height - 1, ymax)

                # Choose color based on class ID
                # if class_id == 0:
                #     color = (0, 0, 0)  # Black
                #     label_text = "Black"
                if class_id == 1:
                    color = (0, 255, 0)  # Green
                    label_text = "Green"
                elif class_id == 2:
                    color = (0, 0, 255)  # Red
                    label_text = "Red"
                else:
                    continue

                cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), color, 2)
                cv2.putText(cv_image, label_text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Draw labeled LiDAR points for red and green objects
        for i in range(len(u)):
            # if labels[i] != 0:
            # if labels[i] == 0:
            #     color = (0, 0, 0)  # Black
            if labels[i] == 1:
                color = (0, 255, 0)  # Green
            elif labels[i] == 2:
                    color = (0, 0, 255)  # red
            else:
                # color = (255, 0, 0)  # Blue
                continue
            
            cv2.circle(cv_image, (u[i], v[i]), 2, color, -1)
        # self.get_logger().info(
        #                         f"Intenisty: {labels}"
        #                     )
        # Save and publish the image
        overlay_image_msg = self.cv_bridge_.cv2_to_imgmsg(cv_image, encoding='bgr8')
        overlay_image_msg.header = image_msg.header
        self.overlay_image_pub_.publish(overlay_image_msg)

        # Create labels_for_scan array
        labels_for_scan = np.zeros(len(scan_msg.ranges), dtype=float)

        # Map labels back to the original scan indices
        labels_for_scan[indices] = labels

        # Create new LaserScan message
        labeled_scan = LaserScan()
        labeled_scan.header = scan_msg.header
        labeled_scan.angle_min = scan_msg.angle_min
        labeled_scan.angle_max = scan_msg.angle_max
        labeled_scan.angle_increment = scan_msg.angle_increment
        labeled_scan.time_increment = scan_msg.time_increment
        labeled_scan.scan_time = scan_msg.scan_time
        labeled_scan.range_min = scan_msg.range_min
        labeled_scan.range_max = scan_msg.range_max
        labeled_scan.ranges = scan_msg.ranges
        labeled_scan.intensities = labels_for_scan.tolist()  # Convert to list
        # self.get_logger().info(
        #                         f"Intenisty: {labeled_scan.intensities}"
        #                     )
        # Publish the labeled scan
        self.labeled_scan_pub_.publish(labeled_scan)

    def compute_lidar_points(self, scan_msg):
        """
        Compute LiDAR points from LaserScan message.
        """
        # Get laser scan data
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(len(ranges)) * scan_msg.angle_increment + scan_msg.angle_min
        indices = np.arange(len(ranges))

        # Filter out invalid ranges
        valid_indices = np.isfinite(ranges) & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        indices = indices[valid_indices]

        # Compute LiDAR points in LiDAR frame
        x_lidar = ranges * np.cos(angles)
        y_lidar = ranges * np.sin(angles)
        z_lidar = np.zeros_like(x_lidar)

        # Stack into (N, 3) array
        points_lidar = np.vstack((x_lidar, y_lidar, z_lidar)).T

        return points_lidar, indices

    def transform_lidar_points_to_camera_frame(self, points_lidar, source_frame, target_frame, time_stamp):
        """
        Transform LiDAR points from source_frame to target_frame (e.g., camera frame).
        """
        try:
            # Use the timestamp from the scan message for accurate transformation
            transform = self.tf_buffer_.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=time_stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

        # Apply transformation
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        rotation = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        rot_matrix = tf_transformations.quaternion_matrix(rotation)[:3, :3]
        points_camera = np.dot(rot_matrix, points_lidar.T).T + translation

        return points_camera
    
    # def transform_lidar_points_to_camera_frame(self, points_lidar, source_frame, target_frame, time_stamp):
    #     """
    #     Transform LiDAR points from source_frame to target_frame (e.g., camera frame).
    #     """
    #     try:
    #         # Use the timestamp from the scan message for accurate transformation
    #         transform = self.tf_buffer_.lookup_transform(
    #             target_frame=target_frame,
    #             source_frame=source_frame,
    #             time=time_stamp,
    #             timeout=rclpy.duration.Duration(seconds=1.0))
    #     except TransformException as ex:
    #         self.get_logger().warn(f'Could not transform {source_frame} to {target_frame}: {ex}')
    #         return None

    #     # Extract translation and rotation from the transform
    #     translation = np.array([
    #         transform.transform.translation.x,
    #         transform.transform.translation.y,
    #         transform.transform.translation.z
    #     ])
    #     rotation = np.array([
    #         transform.transform.rotation.x,
    #         transform.transform.rotation.y,
    #         transform.transform.rotation.z,
    #         transform.transform.rotation.w
    #     ])

    #     # Convert quaternion to rotation matrix using transforms3d
    #     rot_matrix = transforms3d.quaternions.quat2mat(rotation)

    #     # Apply transformation
    #     points_camera = np.dot(rot_matrix, points_lidar.T).T + translation

    #     return points_camera

    def project_points_to_image_plane(self, points_camera, fx, fy, cx, cy, image_msg, indices):
        """
        Project 3D points in camera frame onto the 2D image plane.
        """
        x_cam = points_camera[:, 0]
        y_cam = points_camera[:, 1]
        z_cam = points_camera[:, 2]

        # Avoid division by zero and points behind the camera
        valid_indices = z_cam > 0
        x_cam = x_cam[valid_indices]
        y_cam = y_cam[valid_indices]
        z_cam = z_cam[valid_indices]
        indices = indices[valid_indices]

        

        u = (fx * x_cam / z_cam + cx).astype(int)
        v = (fy * y_cam / z_cam + cy).astype(int)
        

        # Get image dimensions
        height, width = image_msg.height, image_msg.width

        # Filter points that are within the image frame
        valid_indices = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u = u[valid_indices]
        v = v[valid_indices]
        x_cam = x_cam[valid_indices]
        y_cam = y_cam[valid_indices]
        z_cam = z_cam[valid_indices]
        indices = indices[valid_indices]

        return u, v, x_cam, y_cam, z_cam, indices

    def associate_points_with_detections(self, u, v, x_cam, y_cam, z_cam, indices, labels, detections_msg, image_msg):
        """
        Associate projected points with YOLO detections and estimate object positions.
        """
        detections = detections_msg.detections
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = image_msg.header.frame_id  # Or robot base frame

        for idx, detection in enumerate(detections):
            class_id = detection.class_id

            # Process only red and green objects (assuming class IDs 0 and 1)
            if class_id in [1, 2]:
                # Get bounding box coordinates
                bbox = detection.bbox
                image_width = image_msg.width
                image_height = image_msg.height

                # Bounding box coordinates
                xmin = int(bbox.center.position.x - bbox.size.x / 2)
                ymin = int(bbox.center.position.y - bbox.size.y / 2)
                xmax = int(bbox.center.position.x + bbox.size.x / 2)
                ymax = int(bbox.center.position.y + bbox.size.y / 2)

                # Clip coordinates to image dimensions
                xmin = max(0, xmin)
                ymin = max(0, ymin)
                xmax = min(image_width - 1, xmax)
                ymax = min(image_height - 1, ymax)

                # Find points within bounding box
                in_bbox = (u >= xmin) & (u <= xmax) & (v >= ymin) & (v <= ymax)

                if np.any(in_bbox):
                    # Get corresponding 3D points in camera frame
                    x_obj = x_cam[in_bbox]
                    y_obj = y_cam[in_bbox]
                    z_obj = z_cam[in_bbox]

                    # Set labels for these points
                    labels[in_bbox] = class_id

                    # Estimate object position (e.g., mean of points)
                    x_mean = np.mean(x_obj)
                    y_mean = np.mean(y_obj)
                    z_mean = np.mean(z_obj)

                    # Create Pose message
                    pose_msg = Pose()
                    pose_msg.position.x = x_mean
                    pose_msg.position.y = y_mean
                    pose_msg.position.z = z_mean
                    # Orientation can be set if needed, else defaults to zero
                    pose_array_msg.poses.append(pose_msg)

                    # Log object position
                    # self.get_logger().info(
                    #     f"Detection {idx + 1}: Detected Object Position (x, y, z) = "
                    #     f"({x_mean:.2f}, {y_mean:.2f}, {z_mean:.2f}) meters"
                    # )
                else:
                    self.get_logger().warn("No LiDAR points found in bounding box for detection.")
            else:
                # Ignore detections that are not red or green
                continue

        return pose_array_msg, labels

    def caminfoCallback(self, msg: CameraInfo):
        """
        Callback function for handling camera information.
        """
        # Fill self.camera_info_ field
        K = np.array(msg.k)
        if len(K) == 9:
            K = K.reshape((3, 3))
            self.camera_info_ = {'fx': K[0][0], 'fy': K[1][1], 'cx': K[0][2], 'cy': K[1][2]}
            # self.get_logger().info("Camera info received and stored.")
            # self.get_logger().info(f"Camera Intrinsics - fx: {self.camera_info_['fx']}, fy: {self.camera_info_['fy']}, "
            #                        f"cx: {self.camera_info_['cx']}, cy: {self.camera_info_['cy']}")
        else:
            self.get_logger().warn("Invalid camera info received.")

def main(args=None):
    rclpy.init(args=args)
    node = Lidar2ImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
