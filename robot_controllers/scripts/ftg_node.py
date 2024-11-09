#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
import numpy as np
import math
import tf_transformations
import time


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # Initialize parameters
        self.declare_parameter('car_width', 0.5)
        self.declare_parameter('safety_radius', 0.2)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('wheel_base', 0.2255)
        self.declare_parameter('field_of_view', 170.0)  # FOV in degrees
        self.declare_parameter('enable_disparity_extender', True)
        self.declare_parameter('enable_corner_case', True)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('time_horizon', 1.0)  # Time to predict ahead in seconds
        self.declare_parameter('steering_smoothing_factor', 0.1)  # For smoothing steering angles
        self.declare_parameter('use_labeled_scan', True)
        self.declare_parameter('publish_speed', True)
        self.declare_parameter('discontinuity_threshold', 0.3)  # Adjust as needed
        self.declare_parameter('safety_angle_degrees', 15.0)
        self.declare_parameter('best_point_conv_size', 5)
        self.declare_parameter('max_sub_window_size', 10)  # Default: 10 points
        self.declare_parameter('sub_window_step', 1)  # Default: 1 point
        self.declare_parameter('disparity_threshold', 1.0)
        self.declare_parameter('emergency_stop_distance', 0.2)

        # Get parameters
        self.car_width_ = self.get_parameter('car_width').value
        self.safety_radius_ = self.get_parameter('safety_radius').value
        self.max_speed_ = self.get_parameter('max_speed').value
        self.min_speed_ = self.get_parameter('min_speed').value
        self.max_range_ = self.get_parameter('max_range').value
        self.wheel_base_ = self.get_parameter('wheel_base').value
        self.field_of_view_ = self.get_parameter('field_of_view').value
        self.enable_disparity_extender_ = self.get_parameter('enable_disparity_extender').value
        self.enable_corner_case_ = self.get_parameter('enable_corner_case').value
        self.wheel_radius_ = self.get_parameter('wheel_radius').value
        self.use_labeled_scan_ = self.get_parameter('use_labeled_scan').value
        self.publish_speed_ = self.get_parameter('publish_speed').value
        self.discontinuity_threshold_ = self.get_parameter('discontinuity_threshold').value
        self.safety_angle_degrees_ = self.get_parameter('safety_angle_degrees').value
        self.best_point_conv_size_ = self.get_parameter('best_point_conv_size').value
        self.max_sub_window_size_ = self.get_parameter('max_sub_window_size').value
        self.sub_window_step_ = self.get_parameter('sub_window_step').value
        self.disparity_threshold_ = self.get_parameter('disparity_threshold').value
        self.emergency_stop_distance_ = self.get_parameter('emergency_stop_distance').value

        # Initialize processed_scan_
        self.processed_scan_ = LaserScan()
        self.processed_scan_.header.frame_id = 'laser_frame'  # Update as per your frame
        self.processed_scan_.range_min = 0.0  # Will be set based on original scan
        self.processed_scan_.range_max = float(self.max_range_)
        # angle_min, angle_max, angle_increment, and ranges will be set in preprocessLidar

        # Subscribers and Publishers
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scanCallback,
            10
        )

        # Publishers for the steer angle and wheel speed
        self.steer_angle_pub_ = self.create_publisher(Float64, '/steer_angle', 10)
        self.wheel_speed_pub_ = self.create_publisher(Float64, '/drive', 10)

        # Publisher for Gazebo simulation
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for visualization marker
        self.marker_pub_ = self.create_publisher(Marker, '/visualization_marker', 10)

        # Publisher for adjusted LiDAR scans
        self.adjusted_scan_pub_ = self.create_publisher(LaserScan, '/adjusted_scan', 10)

        # Publisher for limited FOV LiDAR scan
        self.limited_fov_scan_pub_ = self.create_publisher(LaserScan, '/limited_fov_scan', 10)

        # Publisher for best gap points
        self.best_gap_points_pub_ = self.create_publisher(Marker, '/best_gap_points', 10)

        # Previous steering angle for smoothing (not used in the given code)
        self.previous_steering_angle_ = 0.0

        self.get_logger().info('FollowTheGap node initialized')

    def scanCallback(self, scan_msg):
        self.get_logger().debug('Received LaserScan message')

        # Start timing
        start_time = time.time()

        # 0- Emergency STOP
        # Emergency stop check before processing
        if self.isEmergencyStop(scan_msg):
            self.publishDriveCommand(0.0, 0.0)
            self.get_logger().warn('Emergency stop activated. Robot stopped.')
            return

        # Step 1: Preprocess the LiDAR data and limit the field of view
        self.get_logger().debug('Step 1: Preprocessing LiDAR data')
        self.preprocessLidar(scan_msg)

        # Publish the scan with limited FOV
        self.publishLimitedFOVScan()

        # Step 2: Apply safety bubble
        self.get_logger().debug('Step 2: Applying safety bubble')
        self.applySafetyBubble()

        if self.use_labeled_scan_:
            # Process the intensities to adjust ranges based on object classes
            self.get_logger().debug('Processing intensities for labeled objects')
            self.processIntensities()

        # Step 3: Optionally apply disparity extender
        if self.enable_disparity_extender_:
            self.get_logger().debug('Step 3: Applying disparity extender')
            self.applyDisparityExtender()

        # Publish the adjusted LiDAR scan after all processing
        self.publishAdjustedScan()

        # Step 4: Find the deepest valid gap that satisfies width constraint
        self.get_logger().debug('Step 4: Finding deepest valid gap that satisfies width constraint')
        gap = self.findBestGap()

        # Publish the best gap points
        self.publishBestGapPoints(gap[0], gap[1], scan_msg.header)

        # Step 5: Check if no valid gap is found
        if gap[0] == 0 and gap[1] == 0:
            self.get_logger().warn('No valid gap found. Stopping the robot.')
            # Publish zero velocity command
            self.publishDriveCommand(0.0, 0.0)
            # Publish visualization marker
            self.publishMarker(0.0, scan_msg.header)

            # End timing
            end_time = time.time()
            duration = end_time - start_time
            self.get_logger().info(f'Execution Time: {duration:.6f} seconds')

            return

        # Step 6: Find the best point in the gap
        self.get_logger().debug('Step 6: Finding best point in the gap')
        best_point_idx = self.findBestPoint(gap[0], gap[1])

        # Step 7: Compute steering angle and speed
        self.get_logger().debug('Step 7: Computing steering angle and speed')
        steering_angle = self.calculateSteeringAngle(best_point_idx)
        speed = self.computeSpeed(scan_msg)

        self.get_logger().info(f'Steering Angle: {steering_angle:.3f} radians, Speed: {speed:.3f} m/s')

        # Step 8: Publish drive command
        if self.publish_speed_:
            self.get_logger().debug('Step 8: Publishing drive command')
            self.publishDriveCommand(steering_angle, speed)

        # Publish visualization marker
        self.publishMarker(steering_angle, scan_msg.header)

        # End timing
        end_time_final = time.time()
        duration_final = end_time_final - start_time
        self.get_logger().info(f'Execution Time: {duration_final:.6f} seconds')

    
    def isEmergencyStop(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]  # Remove NaNs and Infs

        num_ranges = len(ranges)
        front_angle = num_ranges // 2
        angle_range = num_ranges // 5  # +/- 10% around the front

        start_idx = max(front_angle - angle_range, 0)
        end_idx = min(front_angle + angle_range, num_ranges - 1)

        front_ranges = ranges[start_idx:end_idx + 1]
        valid_front_ranges = front_ranges[front_ranges > 0.0]

        if len(valid_front_ranges) > 0:
            min_range = np.min(valid_front_ranges)
        else:
            min_range = float(self.max_range_)

        if min_range < self.emergency_stop_distance_:
            self.get_logger().warn(f'Emergency stop triggered! Obstacle at {min_range:.2f} meters.')
            return True
        else:
            return False

    def preprocessLidar(self, scan_msg):
        # Limit the field of view
        fov_rad = (self.field_of_view_ * math.pi) / 180.0  # Convert FOV to radians
        half_fov = fov_rad / 2.0

        # Calculate desired angle range based on original scan's angle_min
        desired_angle_min = scan_msg.angle_min + (scan_msg.angle_max - scan_msg.angle_min - fov_rad) / 2.0
        desired_angle_max = desired_angle_min + fov_rad

        # Ensure desired angles are within the original scan's range
        desired_angle_min = max(desired_angle_min, scan_msg.angle_min)
        desired_angle_max = min(desired_angle_max, scan_msg.angle_max)

        angle_increment = scan_msg.angle_increment

        # Calculate start and end indices based on desired FOV
        start_idx = int((desired_angle_min - scan_msg.angle_min) / angle_increment)
        end_idx = int((desired_angle_max - scan_msg.angle_min) / angle_increment)

        # Clamp indices to valid range
        start_idx = max(0, start_idx)
        end_idx = min(len(scan_msg.ranges) - 1, end_idx)

        # Extract the limited FOV ranges
        limited_ranges = np.array(scan_msg.ranges[start_idx:end_idx + 1])

        if len(scan_msg.intensities) >= len(scan_msg.ranges):
            limited_intensities = np.array(scan_msg.intensities[start_idx:end_idx + 1])
        else:
            # If intensities are not available or not matching ranges size, fill with zeros
            limited_intensities = np.zeros_like(limited_ranges)

        # Remove NaNs and infs, cap ranges at max_range_
        max_range = float(self.max_range_)
        limited_ranges[np.isnan(limited_ranges) | np.isinf(limited_ranges)] = 0.0
        limited_ranges = np.minimum(limited_ranges, max_range)

        # Update processed_scan_
        self.processed_scan_.header = scan_msg.header
        self.processed_scan_.angle_min = scan_msg.angle_min + (start_idx * angle_increment)
        self.processed_scan_.angle_max = scan_msg.angle_min + (end_idx * angle_increment)
        self.processed_scan_.angle_increment = angle_increment
        self.processed_scan_.time_increment = scan_msg.time_increment
        self.processed_scan_.scan_time = scan_msg.scan_time
        self.processed_scan_.range_min = scan_msg.range_min
        self.processed_scan_.range_max = scan_msg.range_max
        self.processed_scan_.ranges = limited_ranges.tolist()
        self.processed_scan_.intensities = limited_intensities.tolist()

        # Debugging: Log the adjusted angles
        self.get_logger().debug('Preprocessed LiDAR:')
        self.get_logger().debug(f'Adjusted Angle Min: {self.processed_scan_.angle_min:.4f} radians')
        self.get_logger().debug(f'Adjusted Angle Max: {self.processed_scan_.angle_max:.4f} radians')
        self.get_logger().debug(f'Adjusted Angle Increment: {self.processed_scan_.angle_increment:.6f} radians')
        self.get_logger().debug(f'Number of ranges after preprocessing: {len(self.processed_scan_.ranges)}')

    def processIntensities(self):
        if len(self.processed_scan_.intensities) != len(self.processed_scan_.ranges):
            self.get_logger().warn('Intensities size does not match ranges size. Skipping processIntensities.')
            return

        ranges = np.array(self.processed_scan_.ranges)
        intensities = np.array(self.processed_scan_.intensities)
        int_intensities = intensities.astype(int)

        # label == 1 -> green, label == 2 --> parking color(pink), label == 3 --> red
        mask = (int_intensities == 1) | (int_intensities == 2)  | (int_intensities == 3) 

        masked_indices = np.where(mask)[0]

        if len(masked_indices) > 0:
            masked_ranges = ranges[mask]
            min_idx = np.argmin(masked_ranges)
            closest_idx = masked_indices[min_idx]
            min_range = masked_ranges[min_idx]
            label = int_intensities[closest_idx]
            found = True
            self.get_logger().info(f'Closest labeled object at index {closest_idx}, range {min_range:.2f}, label {label}')

            # Modify the ranges based on the label
            # label == 1 -> green, label == 2 --> parking color(pink), label == 3 --> red
            if label == 1:
                # Green object: modify ranges on the right
                ranges[:closest_idx] = 0.0
            elif label == 3:
                # Red object: modify ranges on the left
                ranges[closest_idx + 1:] = 0.0

            self.processed_scan_.ranges = ranges.tolist()
        else:
            self.get_logger().debug('No labeled objects found in intensities.')

    def applySafetyBubble(self):
        ranges = np.array(self.processed_scan_.ranges)
        # Find the closest point
        closest_idx = np.argmin(ranges)

        # Prevent out-of-bounds
        if closest_idx >= len(ranges):
            self.get_logger().warn('Closest index out of bounds. Skipping safety bubble.')
            return

        theta = math.atan2(self.safety_radius_, ranges[closest_idx])

        # Calculate the number of points to clear around the closest point
        bubble_size = int(theta / self.processed_scan_.angle_increment)

        start_idx = closest_idx - bubble_size if closest_idx > bubble_size else 0
        end_idx = min(closest_idx + bubble_size, len(ranges) - 1)

        ranges[start_idx:end_idx + 1] = 0.0

        # Update the ranges
        self.processed_scan_.ranges = ranges.tolist()

        # Log the computed values
        self.get_logger().info(f'Bubble Size: {bubble_size}')
        self.get_logger().info(f'Closest Index: {closest_idx}')
        self.get_logger().info(f'Start Index: {start_idx}')
        self.get_logger().info(f'End Index: {end_idx}')

    def applyDisparityExtender(self):
        ranges = np.array(self.processed_scan_.ranges)
        num_ranges = len(ranges)
        angle_increment = self.processed_scan_.angle_increment
        car_half_width = float(self.car_width_ / 2.0)

        r1 = ranges[:-1]
        r2 = ranges[1:]

        # Skip if any of the ranges are zero (no obstacle)
        valid_mask = (r1 > 0.0) & (r2 > 0.0)
        range_diff = np.abs(r2 - r1)

        disparity_mask = valid_mask & (range_diff > self.disparity_threshold_)

        indices = np.where(disparity_mask)[0]

        for i in indices:
            min_range = min(r1[i], r2[i])
            theta = math.atan2(car_half_width, min_range)
            num_points = int(theta / angle_increment)

            if r1[i] < r2[i]:
                # Extend to the right
                end_idx = min(i + num_points + 1, num_ranges - 1)
                ranges[i + 1:end_idx] = r1[i]
            else:
                # Extend to the left
                start_idx = max(i - num_points + 1, 0)
                ranges[start_idx:i + 1] = r2[i]

            # Debugging: Log disparity extender actions
            self.get_logger().debug(f'Applied disparity extender at index {i}')

        self.processed_scan_.ranges = ranges.tolist()

    def findBestGap(self):
        ranges = np.array(self.processed_scan_.ranges)
        num_ranges = len(ranges)

        valid = (ranges > 0.0) & ~np.isnan(ranges) & ~np.isinf(ranges)
        valid = valid.astype(int)

        diffs = np.diff(valid)

        start_indices = np.where(diffs == 1)[0] + 1
        end_indices = np.where(diffs == -1)[0]

        if valid[0]:
            start_indices = np.insert(start_indices, 0, 0)
        if valid[-1]:
            end_indices = np.append(end_indices, num_ranges - 1)

        if len(start_indices) == 0 or len(end_indices) == 0:
            self.get_logger().warn('No valid gap found (all ranges are zero or invalid).')
            return (0, 0)

        gap_lengths = end_indices - start_indices + 1

        max_gap_idx = np.argmax(gap_lengths)
        max_gap_start = start_indices[max_gap_idx]
        max_gap_end = end_indices[max_gap_idx]
        max_gap_length = gap_lengths[max_gap_idx]

        self.get_logger().debug(f'Best gap found from index {max_gap_start} to {max_gap_end} with length {max_gap_length}')

        return (max_gap_start, max_gap_end)

    def findBestPoint(self, start_idx, end_idx):
        # Apply convolution to stabilize the best point selection
        ranges = np.array(self.processed_scan_.ranges)
        gap_size = end_idx - start_idx

        if gap_size == 0:
            self.get_logger().warn('Gap size is zero. Selecting start index as best point.')
            return start_idx

        # Ensure window size does not exceed gap size
        effective_window_size = min(self.best_point_conv_size_, gap_size)

        averaged_max_gap = np.convolve(ranges[start_idx:end_idx], np.ones(effective_window_size),
                                       'same') / effective_window_size
        best_point_idx = averaged_max_gap.argmax() + start_idx
        self.get_logger().debug(f'Best point in gap at index {best_point_idx} with range {ranges[best_point_idx]:.2f} meters')

        return best_point_idx

    def computeSpeed(self, scan_msg):
        # Compute speed based on minimum distance to obstacles in front
        ranges = np.array(scan_msg.ranges)
        num_ranges = len(ranges)
        front_angle = num_ranges // 2
        angle_range = num_ranges // 10  # Consider +/- 10% of the range around the front

        start_idx = front_angle - angle_range if front_angle >= angle_range else 0
        end_idx = min(front_angle + angle_range, num_ranges - 1)

        front_ranges = ranges[start_idx:end_idx + 1]
        valid_front_ranges = front_ranges[front_ranges > 0.0]

        if len(valid_front_ranges) > 0:
            min_range = np.min(valid_front_ranges)
        else:
            min_range = float(self.max_range_)

        speed = float(self.min_speed_)

        # if min_range > (self.safety_radius_ * 2):
        speed = float(self.min_speed_ + (self.max_speed_ - self.min_speed_) * (
                    (min_range - self.safety_radius_) / (self.max_range_ - self.safety_radius_)))
        speed = min(speed, float(self.max_speed_))
        # else:
        #     speed = 0.0  # Stop if obstacle is too close

        return speed

    def calculateSteeringAngle(self, best_point_idx):
        # Calculate steering angle based on the best point index
        angle = (float(best_point_idx) - (len(self.processed_scan_.ranges) / 2.0)) * self.processed_scan_.angle_increment
        return angle

    def publishDriveCommand(self, steering_angle, speed):
        # Publish steer angle and wheel speed
        steer_angle_msg = Float64()
        steer_angle_msg.data = steering_angle

        wheel_vel_msg = Float64()
        if self.wheel_radius_ > 0:
            wheel_vel_msg.data = speed / self.wheel_radius_
        else:
            self.get_logger().error(f'wheel_radius {self.wheel_radius_:.2f} is not > 0. Defaulting to zero speed')
            wheel_vel_msg.data = 0.0

        self.steer_angle_pub_.publish(steer_angle_msg)
        self.wheel_speed_pub_.publish(wheel_vel_msg)

        # Publish to /cmd_vel topic for Gazebo simulation
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = speed
        cmd_vel_msg.angular.z = self.steeringAngleToAngularVelocity(steering_angle, speed)
        self.cmd_vel_pub_.publish(cmd_vel_msg)

    def steeringAngleToAngularVelocity(self, steering_angle, speed):
        # Convert steering angle and speed to angular velocity for Twist message
        # Using the bicycle model: angular_velocity = speed * tan(steering_angle) / wheel_base

        angular_velocity = 0.0
        if abs(steering_angle) > 0.001:
            angular_velocity = speed * math.tan(steering_angle) / float(self.wheel_base_)
        else:
            angular_velocity = 0.0

        return angular_velocity

    def publishMarker(self, steering_angle, header):
        # Publish a marker to visualize the chosen direction
        marker = Marker()
        marker.header = header
        marker.ns = 'ftg_direction'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Orientation representing the steering angle
        quat = tf_transformations.quaternion_from_euler(0, 0, steering_angle)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        # Set the scale of the marker
        marker.scale.x = 1.0  # Arrow length
        marker.scale.y = 0.1  # Arrow width
        marker.scale.z = 0.1  # Arrow height

        # Set the color
        marker.color.a = 1.0  # Alpha
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Frame ID (LiDAR frame)
        marker.header.frame_id = header.frame_id

        self.marker_pub_.publish(marker)

    def publishLimitedFOVScan(self):
        # Create a new LaserScan message from processed_scan_
        limited_scan = self.processed_scan_  # Copy the processed scan

        # Publish the limited FOV scan
        self.limited_fov_scan_pub_.publish(limited_scan)

        # Debugging: Log the publishing action
        self.get_logger().debug(f'Published limited FOV scan with {len(limited_scan.ranges)} ranges')

    def publishAdjustedScan(self):
        # Create a new LaserScan message from processed_scan_
        adjusted_scan = self.processed_scan_  # Copy the processed scan

        # Publish the adjusted scan
        self.adjusted_scan_pub_.publish(adjusted_scan)

        # Debugging: Log the publishing action
        self.get_logger().debug(f'Published adjusted scan with {len(adjusted_scan.ranges)} ranges')

    def publishBestGapPoints(self, start_idx, end_idx, header):
        points = Marker()
        points.header = header
        points.ns = 'best_gap_points'
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD

        # Set the scale of the points
        points.scale.x = 0.05  # Point width
        points.scale.y = 0.05  # Point height

        # Set the color
        points.color.a = 1.0  # Alpha
        points.color.r = 1.0  # Red color
        points.color.g = 0.0
        points.color.b = 0.0

        # Calculate the angle for each point
        ranges = np.array(self.processed_scan_.ranges)
        angles = self.processed_scan_.angle_min + np.arange(len(ranges)) * self.processed_scan_.angle_increment

        for i in range(start_idx, end_idx + 1):
            if ranges[i] > 0.0:
                p = Point()
                p.x = ranges[i] * math.cos(angles[i])
                p.y = ranges[i] * math.sin(angles[i])
                p.z = 0.0
                points.points.append(p)

        self.best_gap_points_pub_.publish(points)

        # Debugging: Log the number of points published
        self.get_logger().debug(f'Published {len(points.points)} best gap points')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FollowTheGap()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
