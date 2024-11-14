// lidar2image_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yolov8_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <numeric>
#include <algorithm>

class Lidar2ImageNode : public rclcpp::Node
{
public:
    Lidar2ImageNode();

private:
    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<yolov8_msgs::msg::DetectionArray> detections_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<
        yolov8_msgs::msg::DetectionArray,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::LaserScan>
        SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;

    // Camera info subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr object_positions_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlay_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr labeled_scan_pub_;

    // TF buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Camera intrinsics
    double fx_, fy_, cx_, cy_;
    bool camera_info_received_;

    // Callback functions
    void detection_image_scan_callback(
        const yolov8_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg);

    void caminfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    // Helper functions
    void compute_lidar_points(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg,
        std::vector<Eigen::Vector3d> &points_lidar,
        std::vector<size_t> &indices);

    bool transform_lidar_points_to_camera_frame(
        const std::vector<Eigen::Vector3d> &points_lidar,
        const std::string &source_frame,
        const std::string &target_frame,
        const rclcpp::Time &time_stamp,
        std::vector<Eigen::Vector3d> &points_camera);

    void project_points_to_image_plane(
        const std::vector<Eigen::Vector3d> &points_camera,
        double fx, double fy, double cx, double cy,
        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
        const std::vector<size_t> &indices,
        std::vector<int> &u, std::vector<int> &v,
        std::vector<double> &x_cam, std::vector<double> &y_cam, std::vector<double> &z_cam,
        std::vector<size_t> &valid_indices);

    void associate_points_with_detections(
        const std::vector<int> &u, const std::vector<int> &v,
        const std::vector<double> &x_cam, const std::vector<double> &y_cam, const std::vector<double> &z_cam,
        const std::vector<size_t> &valid_indices,
        std::vector<int> &labels,
        const yolov8_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
        geometry_msgs::msg::PoseArray &pose_array_msg);
};

Lidar2ImageNode::Lidar2ImageNode()
    : Node("lidar2image_node"),
      image_sub_(this, "bgr_image"),
      detections_sub_(this, "/yolo/detections"),
      scan_sub_(this, "/scan"),
      sync_(SyncPolicy(100), detections_sub_, image_sub_, scan_sub_),
      camera_info_received_(false)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sync_.registerCallback(std::bind(&Lidar2ImageNode::detection_image_scan_callback, this,
                                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&Lidar2ImageNode::caminfoCallback, this, std::placeholders::_1));

    object_positions_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_object_positions", 10);
    overlay_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("overlay_image", 10);
    labeled_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("labeled_scan", 10);

    RCLCPP_INFO(this->get_logger(), "Initialized lidar2image_node");
}

void Lidar2ImageNode::detection_image_scan_callback(
    const yolov8_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
{
    RCLCPP_INFO(this->get_logger(), "Processing synchronized messages");

    // Ensure camera_info_ is available
    if (!camera_info_received_)
    {
        RCLCPP_WARN(this->get_logger(), "Camera info not yet received. Cannot process data.");
        return;
    }

    // Get laser scan data and compute LiDAR points
    std::vector<Eigen::Vector3d> points_lidar;
    std::vector<size_t> indices;
    compute_lidar_points(scan_msg, points_lidar, indices);

    // Transform LiDAR points to camera frame
    rclcpp::Time time_stamp = scan_msg->header.stamp;
    std::vector<Eigen::Vector3d> points_camera;
    if (!transform_lidar_points_to_camera_frame(points_lidar, scan_msg->header.frame_id,
                                                image_msg->header.frame_id, time_stamp, points_camera))
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform lidar points to camera");
        return;
    }

    // Project points onto image plane
    std::vector<int> u, v;
    std::vector<double> x_cam, y_cam, z_cam;
    std::vector<size_t> valid_indices;
    project_points_to_image_plane(points_camera, fx_, fy_, cx_, cy_, image_msg, indices,
                                  u, v, x_cam, y_cam, z_cam, valid_indices);

    // Initialize labels array
    std::vector<int> labels(u.size(), 0);

    // Associate projected points with detections and estimate object positions
    geometry_msgs::msg::PoseArray pose_array_msg;
    associate_points_with_detections(u, v, x_cam, y_cam, z_cam, valid_indices, labels,
                                     detections_msg, image_msg, pose_array_msg);

    // Publish the object positions
    object_positions_pub_->publish(pose_array_msg);

    // Convert image_msg to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat cv_image = cv_ptr->image;

    // Draw bounding boxes and labels for objects
    for (const auto &detection : detections_msg->detections)
    {
        int class_id = detection.class_id;

        if (class_id == 1 || class_id == 2 || class_id == 3)
        {
            auto bbox = detection.bbox;
            int image_width = image_msg->width;
            int image_height = image_msg->height;

            int xmin = static_cast<int>(bbox.center.position.x - bbox.size.x / 2);
            int ymin = static_cast<int>(bbox.center.position.y - bbox.size.y / 2);
            int xmax = static_cast<int>(bbox.center.position.x + bbox.size.x / 2);
            int ymax = static_cast<int>(bbox.center.position.y + bbox.size.y / 2);

            xmin = std::max(0, xmin);
            ymin = std::max(0, ymin);
            xmax = std::min(image_width - 1, xmax);
            ymax = std::min(image_height - 1, ymax);

            cv::Scalar color;
            std::string label_text;
            if (class_id == 1)
            {
                color = cv::Scalar(0, 255, 0); // Green
                label_text = "Green";
            }
            else if (class_id == 2)
            {
                color = cv::Scalar(203, 192, 255); // Pink
                label_text = "Parking";
            }
            else if (class_id == 3)
            {
                color = cv::Scalar(0, 0, 255); // Red
                label_text = "Red";
            }
            else
            {
                continue;
            }

            cv::rectangle(cv_image, cv::Point(xmin, ymin), cv::Point(xmax, ymax), color, 2);
            cv::putText(cv_image, label_text, cv::Point(xmin, ymin - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.9, color, 2);
        }
    }

    // Draw labeled LiDAR points
    for (size_t i = 0; i < u.size(); ++i)
    {
        cv::Scalar color;
        if (labels[i] == 1)
        {
            color = cv::Scalar(0, 255, 0); // Green
        }
        else if (labels[i] == 2)
        {
            color = cv::Scalar(203, 192, 255); // Pink
        }
        else if (labels[i] == 3)
        {
            color = cv::Scalar(0, 0, 255); // Red
        }
        else
        {
            continue;
        }
        cv::circle(cv_image, cv::Point(u[i], v[i]), 2, color, -1);
    }

    // Publish the overlay image
    sensor_msgs::msg::Image::SharedPtr overlay_image_msg = cv_bridge::CvImage(
                                                              image_msg->header, sensor_msgs::image_encodings::BGR8, cv_image)
                                                              .toImageMsg();
    overlay_image_pub_->publish(*overlay_image_msg);

    // Create labels_for_scan array
    std::vector<float> labels_for_scan(scan_msg->ranges.size(), 0.0f);

    // Map labels back to the original scan indices
    for (size_t i = 0; i < valid_indices.size(); ++i)
    {
        size_t idx = indices[valid_indices[i]];
        labels_for_scan[idx] = static_cast<float>(labels[i]);
    }

    // Create new LaserScan message
    sensor_msgs::msg::LaserScan labeled_scan = *scan_msg;
    labeled_scan.intensities = labels_for_scan;

    // Publish the labeled scan
    labeled_scan_pub_->publish(labeled_scan);
}

void Lidar2ImageNode::compute_lidar_points(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg,
    std::vector<Eigen::Vector3d> &points_lidar,
    std::vector<size_t> &indices)
{
    const auto &ranges = scan_msg->ranges;
    double angle = scan_msg->angle_min;

    for (size_t i = 0; i < ranges.size(); ++i)
    {
        double r = ranges[i];
        if (std::isfinite(r) && r >= scan_msg->range_min && r <= scan_msg->range_max)
        {
            double x = r * std::cos(angle);
            double y = r * std::sin(angle);
            double z = 0.0;
            points_lidar.emplace_back(x, y, z);
            indices.push_back(i);
        }
        angle += scan_msg->angle_increment;
    }
}

bool Lidar2ImageNode::transform_lidar_points_to_camera_frame(
    const std::vector<Eigen::Vector3d> &points_lidar,
    const std::string &source_frame,
    const std::string &target_frame,
    const rclcpp::Time &time_stamp,
    std::vector<Eigen::Vector3d> &points_camera)
{
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform(
            target_frame, source_frame, time_stamp, rclcpp::Duration::from_seconds(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }

    Eigen::Translation3d translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);

    Eigen::Quaterniond rotation(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z);

    Eigen::Affine3d transform_matrix = translation * rotation;

    for (const auto &point : points_lidar)
    {
        Eigen::Vector3d point_transformed = transform_matrix * point;
        points_camera.push_back(point_transformed);
    }
    return true;
}

void Lidar2ImageNode::project_points_to_image_plane(
    const std::vector<Eigen::Vector3d> &points_camera,  // Use Eigen::Vector3d instead of Matrix<double, 3, 1>
    double fx, double fy, double cx, double cy,
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
    const std::vector<size_t> &indices,
    std::vector<int> &u, std::vector<int> &v,
    std::vector<double> &x_cam, std::vector<double> &y_cam, std::vector<double> &z_cam,
    std::vector<size_t> &valid_indices)
{
    int width = image_msg->width;
    int height = image_msg->height;

    for (const auto& point : points_camera) {
        double x = point.x();  // Access the x component
        double y = point.y();  // Access the y component
        double z = point.z();  // Access the z component

        if (z > 0) {  // Only consider points in front of the camera
            int ui = static_cast<int>(fx * x / z + cx);
            int vi = static_cast<int>(fy * y / z + cy);

            if (ui >= 0 && ui < width && vi >= 0 && vi < height) {
                u.push_back(ui);
                v.push_back(vi);
                x_cam.push_back(x);
                y_cam.push_back(y);
                z_cam.push_back(z);
                valid_indices.push_back(&point - &points_camera[0]);  // Index of the point
            }
        }
    }
}


void Lidar2ImageNode::associate_points_with_detections(
    const std::vector<int> &u, const std::vector<int> &v,
    const std::vector<double> &x_cam, const std::vector<double> &y_cam, const std::vector<double> &z_cam,
    const std::vector<size_t> &valid_indices,
    std::vector<int> &labels,
    const yolov8_msgs::msg::DetectionArray::ConstSharedPtr &detections_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg,
    geometry_msgs::msg::PoseArray &pose_array_msg)
{
    pose_array_msg.header.stamp = this->now();
    pose_array_msg.header.frame_id = image_msg->header.frame_id;

    for (const auto &detection : detections_msg->detections)
    {
        int class_id = detection.class_id;

        if (class_id == 1 || class_id == 2 || class_id == 3)
        {
            auto bbox = detection.bbox;
            int image_width = image_msg->width;
            int image_height = image_msg->height;

            int xmin = static_cast<int>(bbox.center.position.x - bbox.size.x / 2);
            int ymin = static_cast<int>(bbox.center.position.y - bbox.size.y / 2);
            int xmax = static_cast<int>(bbox.center.position.x + bbox.size.x / 2);
            int ymax = static_cast<int>(bbox.center.position.y + bbox.size.y / 2);

            xmin = std::max(0, xmin);
            ymin = std::max(0, ymin);
            xmax = std::min(image_width - 1, xmax);
            ymax = std::min(image_height - 1, ymax);

            // Find points within bounding box
            std::vector<double> x_obj, y_obj, z_obj;

            for (size_t i = 0; i < u.size(); ++i)
            {
                if (u[i] >= xmin && u[i] <= xmax && v[i] >= ymin && v[i] <= ymax)
                {
                    x_obj.push_back(x_cam[i]);
                    y_obj.push_back(y_cam[i]);
                    z_obj.push_back(z_cam[i]);
                    labels[i] = class_id;
                }
            }

            if (!x_obj.empty())
            {
                // Estimate object position
                double x_mean = std::accumulate(x_obj.begin(), x_obj.end(), 0.0) / x_obj.size();
                double y_mean = std::accumulate(y_obj.begin(), y_obj.end(), 0.0) / y_obj.size();
                double z_mean = std::accumulate(z_obj.begin(), z_obj.end(), 0.0) / z_obj.size();

                geometry_msgs::msg::Pose pose_msg;
                pose_msg.position.x = x_mean;
                pose_msg.position.y = y_mean;
                pose_msg.position.z = z_mean;
                pose_array_msg.poses.push_back(pose_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No LiDAR points found in bounding box for detection.");
            }
        }
    }
}

void Lidar2ImageNode::caminfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    const auto &K = msg->k;
    if (K.size() == 9)
    {
        fx_ = K[0];
        fy_ = K[4];
        cx_ = K[2];
        cy_ = K[5];
        camera_info_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Camera info received and stored.");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info received.");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Lidar2ImageNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
