// src/ftg_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/float64.hpp"

#include <algorithm>
#include <vector>
#include <cmath>

class FollowTheGap : public rclcpp::Node
{
public:
    FollowTheGap() : Node("follow_the_gap"), previous_steering_angle_(0.0f)
    {
        // Initialize parameters
        this->declare_parameter<double>("car_width", 0.5);
        this->declare_parameter<double>("safety_radius", 0.2);
        this->declare_parameter<double>("max_speed", 2.0);
        this->declare_parameter<double>("min_speed", 0.5);
        this->declare_parameter<double>("max_range", 3.0);
        this->declare_parameter<double>("wheel_base", 0.2255);
        this->declare_parameter<double>("field_of_view", 170.0); // FOV in degrees
        this->declare_parameter<bool>("enable_disparity_extender", true);
        this->declare_parameter<bool>("enable_corner_case", true);
        this->declare_parameter<double>("wheel_radius", 0.03);
        this->declare_parameter<double>("time_horizon", 1.0); // Time to predict ahead in seconds
        this->declare_parameter<double>("steering_smoothing_factor", 0.1); // For smoothing steering angles
        this->declare_parameter<bool>("use_labeled_scan", true);
        this->declare_parameter<bool>("publish_speed", true);
        this->declare_parameter<double>("discontinuity_threshold", 0.3); // Adjust as needed
        this->declare_parameter<double>("safety_angle_degrees", 15.0);
        this->declare_parameter<int>("best_point_conv_size", 5);
        this->declare_parameter<int>("max_sub_window_size", 10); // Default: 10 points
        this->declare_parameter<int>("sub_window_step", 1); // Default: 1 point
        


        // Get parameters
        car_width_ = this->get_parameter("car_width").as_double();
        safety_radius_ = this->get_parameter("safety_radius").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        field_of_view_ = this->get_parameter("field_of_view").as_double();
        enable_disparity_extender_ = this->get_parameter("enable_disparity_extender").as_bool();
        enable_corner_case_ = this->get_parameter("enable_corner_case").as_bool();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        use_labeled_scan_ = this->get_parameter("use_labeled_scan").as_bool();
        publish_speed_ = this->get_parameter("publish_speed").as_bool();
        discontinuity_threshold_ = this->get_parameter("discontinuity_threshold").as_double();
        safety_angle_degrees_ = this->get_parameter("safety_angle_degrees").as_double();
        best_point_conv_size_ = static_cast<size_t>(this->get_parameter("best_point_conv_size").as_int());
        max_sub_window_size_ = static_cast<size_t>(this->get_parameter("max_sub_window_size").as_int());
        sub_window_step_ = static_cast<size_t>(this->get_parameter("sub_window_step").as_int());

        // Initialize processed_scan_
        processed_scan_.header.frame_id = "laser_frame"; // Update as per your frame
        processed_scan_.range_min = 0.0; // Will be set based on original scan
        processed_scan_.range_max = static_cast<float>(max_range_);
        // processed_scan_.angle_min, angle_max, angle_increment, and ranges will be set in preprocessLidar

        // Subscribers and Publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FollowTheGap::scanCallback, this, std::placeholders::_1));

        // Publishers for the steer angle and wheel speed
        steer_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steer_angle", 10);
        wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/drive", 10);

        // Publisher for Gazebo simulation
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Publisher for visualization marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker", 10);

        // Publisher for adjusted LiDAR scans
        adjusted_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/adjusted_scan", 10);

        // Publisher for limited FOV LiDAR scan
        limited_fov_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/limited_fov_scan", 10);

        // Publisher for best gap points
        best_gap_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/best_gap_points", 10);

        RCLCPP_INFO(this->get_logger(), "FollowTheGap node initialized");
    }

private:
    // Parameters
    double car_width_;
    double safety_radius_;
    double wheel_radius_;
    double max_speed_;
    double min_speed_;
    double max_range_;
    double wheel_base_;
    double field_of_view_;
    bool enable_disparity_extender_;
    bool enable_corner_case_;
    bool use_labeled_scan_;
    bool publish_speed_ ;
    double discontinuity_threshold_;
    double safety_angle_degrees_;
    size_t best_point_conv_size_;
    size_t max_sub_window_size_;
    size_t sub_window_step_;

    // Processed LaserScan
    sensor_msgs::msg::LaserScan processed_scan_;

    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publishers for the steer angle and wheel speed
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_speed_pub_;

    // Publisher for Gazebo simulation
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Publisher for visualization marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Publisher for adjusted LiDAR scan
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr adjusted_scan_pub_;

    // Publisher for limited FOV LiDAR scan
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr limited_fov_scan_pub_;

    // Publisher for best gap points
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_gap_points_pub_;

    // Previous steering angle for smoothing
    float previous_steering_angle_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received LaserScan message");

        // Start timing
        auto start_time = this->now();

        // Step 1: Preprocess the LiDAR data and limit the field of view
        RCLCPP_DEBUG(this->get_logger(), "Step 1: Preprocessing LiDAR data");
        preprocessLidar(scan_msg);

        // Publish the scan with limited FOV
        publishLimitedFOVScan();

        // Step 2: Apply safety bubble
        RCLCPP_DEBUG(this->get_logger(), "Step 2: Applying safety bubble");
        applySafetyBubble();

        if (use_labeled_scan_)
        {
            // Process the intensities to adjust ranges based on object classes
            RCLCPP_DEBUG(this->get_logger(), "Processing intensities for labeled objects");
            processIntensities();
        }

        // Step 3: Optionally apply disparity extender
        if (enable_disparity_extender_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Step 3: Applying disparity extender");
            applyDisparityExtender();
        }

        // Publish the adjusted LiDAR scan after all processing
        publishAdjustedScan();

        // Step 4: Find the deepest valid gap that satisfies width constraint
        RCLCPP_DEBUG(this->get_logger(), "Step 4: Finding deepest valid gap that satisfies width constraint");
        auto gap = findBestGap();
        // auto sub_gap = findDeepestSubWindow(gap.first, gap.second);

        // Publish the best gap points
        publishBestGapPoints(gap.first, gap.second, scan_msg->header);

        // Step 5: Check if no valid gap is found
        if (gap.first == 0 && gap.second == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No valid gap found. Stopping the robot.");
            // Publish zero velocity command
            publishDriveCommand(0.0f, 0.0f);

            // Publish visualization marker
            publishMarker(0.0f, scan_msg->header);

            // End timing
            auto end_time = this->now();
            auto duration = end_time - start_time;
            RCLCPP_INFO(this->get_logger(), "Execution Time: %.6f seconds", duration.seconds());

            return;
        }

        // Step 6: Find the best point in the gap
        RCLCPP_DEBUG(this->get_logger(), "Step 6: Finding best point in the gap");
        size_t best_point_idx = findBestPoint(gap.first, gap.second);

        // Step 7: Compute steering angle and speed
        RCLCPP_DEBUG(this->get_logger(), "Step 7: Computing steering angle and speed");
        float steering_angle = calculateSteeringAngle(best_point_idx);
        float speed = computeSpeed();

        RCLCPP_INFO(this->get_logger(), "Steering Angle: %.3f radians, Speed: %.3f m/s", steering_angle, speed);

        // Step 8: Publish drive command
        if (publish_speed_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Step 8: Publishing drive command");
            publishDriveCommand(steering_angle, speed);
        }

        // Publish visualization marker
        publishMarker(steering_angle, scan_msg->header);

        // End timing
        auto end_time_final = this->now();
        auto duration_final = end_time_final - start_time;
        RCLCPP_INFO(this->get_logger(), "Execution Time: %.6f seconds", duration_final.seconds());
    }

    void preprocessLidar(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Limit the field of view
        double fov_rad = (field_of_view_ * M_PI) / 180.0; // Convert FOV to radians
        double half_fov = fov_rad / 2.0;

        // Calculate desired angle range based on original scan's angle_min
        double desired_angle_min = scan_msg->angle_min + (scan_msg->angle_max - scan_msg->angle_min - fov_rad) / 2.0;
        double desired_angle_max = desired_angle_min + fov_rad;

        // Ensure desired angles are within the original scan's range
        if (desired_angle_min < scan_msg->angle_min)
            desired_angle_min = scan_msg->angle_min;

        if (desired_angle_max > scan_msg->angle_max)
            desired_angle_max = scan_msg->angle_max;

        double angle_increment = scan_msg->angle_increment;

        // Calculate start and end indices based on desired FOV
        int start_idx = static_cast<int>((desired_angle_min - scan_msg->angle_min) / angle_increment);
        int end_idx = static_cast<int>((desired_angle_max - scan_msg->angle_min) / angle_increment);

        // Clamp indices to valid range
        start_idx = std::max(0, start_idx);
        end_idx = std::min(static_cast<int>(scan_msg->ranges.size()) - 1, end_idx);

        // Extract the limited FOV ranges
        std::vector<float> limited_ranges(scan_msg->ranges.begin() + start_idx, scan_msg->ranges.begin() + end_idx + 1);

        std::vector<float> limited_intensities;
        if (scan_msg->intensities.size() >= scan_msg->ranges.size())
        {
            limited_intensities.assign(scan_msg->intensities.begin() + start_idx, scan_msg->intensities.begin() + end_idx + 1);
        }
        else
        {
            // If intensities are not available or not matching ranges size, fill with zeros
            limited_intensities.assign(limited_ranges.size(), 0.0f);
        }

        // Remove NaNs and infs, cap ranges at max_range_
        float max_range = static_cast<float>(max_range_);
        for (float &range : limited_ranges)
        {
            if (std::isnan(range) || std::isinf(range))
            {
                range = 0.0f;
            }
            if (range > max_range) // clipping
                range = max_range;
        }

        // Update processed_scan_
        processed_scan_.header = scan_msg->header;
        processed_scan_.angle_min = scan_msg->angle_min + (start_idx * angle_increment);
        processed_scan_.angle_max = scan_msg->angle_min + (end_idx * angle_increment);
        processed_scan_.angle_increment = angle_increment;
        processed_scan_.time_increment = scan_msg->time_increment;
        processed_scan_.scan_time = scan_msg->scan_time;
        processed_scan_.range_min = scan_msg->range_min;
        processed_scan_.range_max = scan_msg->range_max;
        processed_scan_.ranges = limited_ranges;
        processed_scan_.intensities = limited_intensities;

        // Debugging: Log the adjusted angles
        RCLCPP_DEBUG(this->get_logger(), "Preprocessed LiDAR:");
        RCLCPP_DEBUG(this->get_logger(), "Adjusted Angle Min: %.4f radians", processed_scan_.angle_min);
        RCLCPP_DEBUG(this->get_logger(), "Adjusted Angle Max: %.4f radians", processed_scan_.angle_max);
        RCLCPP_DEBUG(this->get_logger(), "Adjusted Angle Increment: %.6f radians", processed_scan_.angle_increment);
        RCLCPP_DEBUG(this->get_logger(), "Number of ranges after preprocessing: %zu", processed_scan_.ranges.size());
    }

    
    void processIntensities()
    {
        if (processed_scan_.intensities.size() != processed_scan_.ranges.size())
        {
            RCLCPP_WARN(this->get_logger(), "Intensities size does not match ranges size. Skipping processIntensities.");
            return;
        }

        size_t closest_idx = 0;
        float min_range = 2.0;//std::numeric_limits<float>::max();
        int label = 0; // 1 for green, 2 for red

        bool found = false;

        for (size_t i = 0; i < processed_scan_.ranges.size(); ++i)
        {
            float range = processed_scan_.ranges[i];
            float intensity = processed_scan_.intensities[i];

            // Check if intensity is 1 or 2
            int int_intensity = static_cast<int>(intensity + 0.5f); // Round to nearest int

            if ((int_intensity == 1 || int_intensity == 2) && range > 0.0f)
            {
                if (range < min_range)
                {
                    min_range = range;
                    closest_idx = i;
                    label = int_intensity;
                    found = true;
                }
            }
        }

        if (found)
        {
            RCLCPP_INFO(this->get_logger(), "Closest labeled object at index %zu, range %.2f, label %d", closest_idx, min_range, label);

            // Modify the ranges based on the label
            if (label == 1)
            {
                // Green object: modify ranges on the right
                for (size_t i = 0; i < closest_idx; ++i)
                {
                    processed_scan_.ranges[i] = 0.0; //min_range;
                }
            }
            else if (label == 2)
            {
                // Red object: modify ranges on the left
                for (size_t i = closest_idx + 1; i < processed_scan_.ranges.size(); ++i)
                {
                    processed_scan_.ranges[i] = 0.0; //min_range;
                }
            }
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "No labeled objects found in intensities.");
        }
    }

    void applySafetyBubble()
    {
        // Find the closest point
        size_t closest_idx = std::min_element(processed_scan_.ranges.begin(), processed_scan_.ranges.end()) - processed_scan_.ranges.begin();

        // Prevent out-of-bounds
        if (closest_idx >= processed_scan_.ranges.size())
        {
            RCLCPP_WARN(this->get_logger(), "Closest index out of bounds. Skipping safety bubble.");
            return;
        }

        // float theta = std::asin(ratio);
        float theta = std::atan2(safety_radius_, processed_scan_.ranges[closest_idx]);

        // Calculate the number of points to clear around the closest point
        size_t bubble_size = static_cast<size_t>(theta / processed_scan_.angle_increment);

        size_t start_idx = (closest_idx > bubble_size) ? (closest_idx - bubble_size) : 0;
        size_t end_idx = std::min(closest_idx + bubble_size, processed_scan_.ranges.size() - 1);

        std::fill(processed_scan_.ranges.begin() + start_idx, processed_scan_.ranges.begin() + end_idx + 1, 0.0f);

        // Log the computed values
        RCLCPP_INFO(this->get_logger(), "Bubble Size: %zu", bubble_size);
        RCLCPP_INFO(this->get_logger(), "Closest Index: %zu", closest_idx);
        RCLCPP_INFO(this->get_logger(), "Start Index: %zu", start_idx);
        RCLCPP_INFO(this->get_logger(), "End Index: %zu", end_idx);
    }

    void applyDisparityExtender()
    {
        size_t num_ranges = processed_scan_.ranges.size();
        float angle_increment = processed_scan_.angle_increment;
        float car_half_width = static_cast<float>(car_width_ / 2.0);

        for (size_t i = 0; i < num_ranges - 1; ++i)
        {
            float r1 = processed_scan_.ranges[i];
            float r2 = processed_scan_.ranges[i + 1];

            // Skip if any of the ranges are zero (no obstacle)
            if (r1 == 0.0f || r2 == 0.0f)
                continue;

            float range_diff = std::abs(r2 - r1);

            if (range_diff > 1.0) // TODO: Make this a parameter (e.g., declare_parameter)
            {
                // Calculate the number of points to extend
                float min_range = std::min(r1, r2);

                float theta = std::atan2(car_half_width, min_range);
                size_t num_points = static_cast<size_t>(theta / angle_increment);

                if (r1 < r2)
                {
                    // Extend to the right
                    size_t end_idx = std::min(i + num_points, num_ranges - 1);
                    std::fill(processed_scan_.ranges.begin() + i + 1, processed_scan_.ranges.begin() + end_idx + 1, r1);
                }
                else
                {
                    // Extend to the left
                    size_t start_idx = (i >= num_points) ? (i - num_points + 1) : 0;
                    std::fill(processed_scan_.ranges.begin() + start_idx, processed_scan_.ranges.begin() + i + 1, r2);
                }

                // Debugging: Log disparity extender actions
                RCLCPP_DEBUG(this->get_logger(), "Applied disparity extender at index %zu", i);
            }
        }
    }

    // Updated findBestGap function
    std::pair<size_t, size_t> findBestGap()
    {
        const auto& ranges = processed_scan_.ranges;
        size_t num_ranges = ranges.size();

        size_t max_gap_length = 0;
        size_t max_gap_start = 0;
        size_t max_gap_end = 0;

        size_t current_gap_start = 0;
        size_t current_gap_length = 0;

        for (size_t i = 0; i < num_ranges; ++i)
        {
            if (ranges[i] > 0.0f && !std::isnan(ranges[i]) && !std::isinf(ranges[i]))
            {
                if (current_gap_length == 0)
                {
                    current_gap_start = i;
                }
                current_gap_length++;
            }
            else
            {
                if (current_gap_length > max_gap_length)
                {
                    max_gap_length = current_gap_length;
                    max_gap_start = current_gap_start;
                    max_gap_end = i - 1;
                }
                current_gap_length = 0;
            }
        }

        // Check at the end of the loop
        if (current_gap_length > max_gap_length)
        {
            max_gap_length = current_gap_length;
            max_gap_start = current_gap_start;
            max_gap_end = num_ranges - 1;
        }

        if (max_gap_length == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No valid gap found (all ranges are zero or invalid).");
            return {0, 0};
        }

        RCLCPP_DEBUG(this->get_logger(), "Best gap found from index %zu to %zu with length %zu", 
                     max_gap_start, max_gap_end, max_gap_length);

        return {max_gap_start, max_gap_end};
    }

    // New method: findDeepestSubWindow
    std::pair<size_t, size_t> findDeepestSubWindow(size_t gap_start, size_t gap_end)
    {
        const auto& ranges = processed_scan_.ranges;
        size_t gap_size = gap_end - gap_start + 1;

        if (gap_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Gap size is zero. Cannot find sub-window.");
            return {0, 0};
        }

        // Determine the effective window size
        size_t window_size = std::min(max_sub_window_size_, gap_size);

        if (window_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Maximum sub-window size is zero. Cannot find sub-window.");
            return {0, 0};
        }

        size_t best_sub_start = 0;
        size_t best_sub_end = 0;
        float max_avg_range = -1.0f;

        // Slide the window across the gap
        for (size_t start = gap_start; start + window_size <= gap_end + 1; start += sub_window_step_)
        {
            size_t end = start + window_size - 1;

            // Calculate the average range within the window
            float sum = 0.0f;
            size_t valid_points = 0;

            for (size_t i = start; i <= end; ++i)
            {
                if (ranges[i] > 0.0f && !std::isnan(ranges[i]) && !std::isinf(ranges[i]))
                {
                    sum += ranges[i];
                    valid_points++;
                }
            }

            if (valid_points == 0)
            {
                RCLCPP_DEBUG(this->get_logger(), "Window from %zu to %zu has no valid points. Skipping.", start, end);
                continue;
            }

            float avg_range = sum / static_cast<float>(valid_points);

            RCLCPP_DEBUG(this->get_logger(), "Window from %zu to %zu has average range %.2f meters.", start, end, avg_range);

            // Update the best sub-window if a deeper average range is found
            if (avg_range > max_avg_range)
            {
                max_avg_range = avg_range;
                best_sub_start = start;
                best_sub_end = end;
            }
        }

        // Handle the case where the gap size is smaller than the window size
        if (best_sub_start == 0 && best_sub_end == 0 && window_size < gap_size)
        {
            // Select the entire gap as the sub-window
            best_sub_start = gap_start;
            best_sub_end = gap_end;

            // Recalculate the average range
            float sum = 0.0f;
            size_t valid_points = 0;

            for (size_t i = best_sub_start; i <= best_sub_end; ++i)
            {
                if (ranges[i] > 0.0f && !std::isnan(ranges[i]) && !std::isinf(ranges[i]))
                {
                    sum += ranges[i];
                    valid_points++;
                }
            }

            if (valid_points > 0)
            {
                max_avg_range = sum / static_cast<float>(valid_points);
                RCLCPP_DEBUG(this->get_logger(), "Selected entire gap from %zu to %zu with average range %.2f meters.", 
                             best_sub_start, best_sub_end, max_avg_range);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Selected entire gap has no valid points.");
                return {0, 0};
            }
        }

        if (max_avg_range == -1.0f)
        {
            RCLCPP_WARN(this->get_logger(), "No valid sub-window found within the gap.");
            return {0, 0};
        }

        RCLCPP_INFO(this->get_logger(), "Selected sub-window from index %zu to %zu with average range %.2f meters.",
                    best_sub_start, best_sub_end, max_avg_range);

        return {best_sub_start, best_sub_end};
    }

 


    // Updated findBestPoint function
    size_t findBestPoint(size_t start_idx, size_t end_idx)
    {
        const auto& ranges = processed_scan_.ranges;
        size_t gap_size = end_idx - start_idx;

        if (gap_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Gap size is zero. Selecting start index as best point.");
            return start_idx;
        }

        // Ensure window size does not exceed gap size
        size_t effective_window_size = std::min(best_point_conv_size_, gap_size);

        // Calculate sliding window averages
        std::vector<float> averaged_max_gap(gap_size, 0.0f);

        for (size_t i = 0; i < gap_size; ++i)
        {
            size_t window_start = (i >= effective_window_size / 2) ? (i - effective_window_size / 2) : 0;
            size_t window_end = std::min(i + effective_window_size / 2, gap_size - 1);

            float sum = 0.0f;
            size_t count = 0;

            for (size_t j = window_start; j <= window_end; ++j)
            {
                sum += ranges[start_idx + j];
                count++;
            }

            averaged_max_gap[i] = sum / static_cast<float>(count);
        }

        // Find the index with the maximum average
        auto max_iter = std::max_element(averaged_max_gap.begin(), averaged_max_gap.end());
        size_t max_index = std::distance(averaged_max_gap.begin(), max_iter);

        size_t best_point_idx = start_idx + max_index;

        RCLCPP_DEBUG(this->get_logger(), "Best point in gap at index %zu with range %.2f meters",
                     best_point_idx, ranges[best_point_idx]);

        return best_point_idx;
    }

    float computeSpeed()
    {
        // Compute speed based on minimum distance to obstacles in front
        size_t num_ranges = processed_scan_.ranges.size();
        size_t front_angle = num_ranges / 2;
        size_t angle_range = num_ranges / 10; // Consider +/- 10% of the range around the front

        size_t start_idx = (front_angle >= angle_range) ? (front_angle - angle_range) : 0;
        size_t end_idx = std::min(front_angle + angle_range, num_ranges - 1);

        float min_range = static_cast<float>(max_range_);
        for (size_t i = start_idx; i <= end_idx; ++i)
        {
            if (processed_scan_.ranges[i] > 0.0f && processed_scan_.ranges[i] < min_range)
            {
                min_range = processed_scan_.ranges[i];
            }
        }

        float speed = static_cast<float>(min_speed_);

        if (min_range > (safety_radius_*2))
        {
            speed = static_cast<float>(min_speed_ + (max_speed_ - min_speed_) * ((min_range - safety_radius_) / (max_range_ - safety_radius_)));
            speed = std::min(speed, static_cast<float>(max_speed_));
        }
        else
        {
            speed = 0.0f; // Stop if obstacle is too close
        }

        return speed;
    }

    float calculateSteeringAngle(size_t best_point_idx)
    {
        // Calculate steering angle based on the best point index
        // Angle = angle_min + index * angle_increment
        // float angle = processed_scan_.angle_min + (static_cast<float>(best_point_idx) * processed_scan_.angle_increment);
        float angle = (static_cast<float>(best_point_idx) - (processed_scan_.ranges.size() / 2) )  * processed_scan_.angle_increment;
        // angle /= 1.5; // Scale down for smoother steering
        return angle;
    }

    void publishDriveCommand(float steering_angle, float speed)
    {
        // Publish steer angle and wheel speed
        auto steer_angle_msg = std_msgs::msg::Float64();
        steer_angle_msg.data = steering_angle;

        auto wheel_vel_msg = std_msgs::msg::Float64();
        if (wheel_radius_ > 0)
        {
            wheel_vel_msg.data = speed / wheel_radius_;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "wheel_radius %.2f is not > 0. Defaulting to zero speed", wheel_radius_);
            wheel_vel_msg.data = 0.0;
        }

        steer_angle_pub_->publish(steer_angle_msg);
        wheel_speed_pub_->publish(wheel_vel_msg);

        // Publish to /cmd_vel topic for Gazebo simulation
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = speed;
        cmd_vel_msg.angular.z = steeringAngleToAngularVelocity(steering_angle, speed);
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    float steeringAngleToAngularVelocity(float steering_angle, float speed)
    {
        // Convert steering angle and speed to angular velocity for Twist message
        // Using the bicycle model: angular_velocity = speed * tan(steering_angle) / wheel_base

        float angular_velocity = 0.0f;
        if (std::abs(steering_angle) > 0.001)
        {
            angular_velocity = speed * std::tan(steering_angle) / static_cast<float>(wheel_base_);
        }
        else
        {
            angular_velocity = 0.0f;
        }

        return angular_velocity;
    }

    void publishMarker(float steering_angle, const std_msgs::msg::Header &header)
    {
        // Publish a marker to visualize the chosen direction
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "ftg_direction";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        // Orientation representing the steering angle
        tf2::Quaternion quat;
        quat.setRPY(0, 0, steering_angle);
        marker.pose.orientation = tf2::toMsg(quat);

        // Set the scale of the marker
        marker.scale.x = 1.0; // Arrow length
        marker.scale.y = 0.1; // Arrow width
        marker.scale.z = 0.1; // Arrow height

        // Set the color
        marker.color.a = 1.0; // Alpha
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // Frame ID (LiDAR frame)
        marker.header.frame_id = header.frame_id;

        marker_pub_->publish(marker);
    }

    void publishLimitedFOVScan()
    {
        // Create a new LaserScan message from processed_scan_
        auto limited_scan = processed_scan_; // Copy the processed scan

        // Publish the limited FOV scan
        limited_fov_scan_pub_->publish(limited_scan);

        // Debugging: Log the publishing action
        RCLCPP_DEBUG(this->get_logger(), "Published limited FOV scan with %zu ranges", limited_scan.ranges.size());
    }

    void publishAdjustedScan()
    {
        // Create a new LaserScan message from processed_scan_
        auto adjusted_scan = processed_scan_; // Copy the processed scan

        // Publish the adjusted scan
        adjusted_scan_pub_->publish(adjusted_scan);

        // Debugging: Log the publishing action
        RCLCPP_DEBUG(this->get_logger(), "Published adjusted scan with %zu ranges", adjusted_scan.ranges.size());
    }

    void publishBestGapPoints(size_t start_idx, size_t end_idx, const std_msgs::msg::Header &header)
    {
        visualization_msgs::msg::Marker points;
        points.header = header;
        points.ns = "best_gap_points";
        points.id = 0;
        points.type = visualization_msgs::msg::Marker::POINTS;
        points.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the points
        points.scale.x = 0.05; // Point width
        points.scale.y = 0.05; // Point height

        // Set the color
        points.color.a = 1.0; // Alpha
        points.color.r = 1.0; // Red color
        points.color.g = 0.0;
        points.color.b = 0.0;

        // Calculate the angle for each point
        double angle = processed_scan_.angle_min + start_idx * processed_scan_.angle_increment;
        for (size_t i = start_idx; i <= end_idx; ++i)
        {
            if (processed_scan_.ranges[i] > 0.0f)
            {
                geometry_msgs::msg::Point p;
                p.x = processed_scan_.ranges[i] * std::cos(angle);
                p.y = processed_scan_.ranges[i] * std::sin(angle);
                p.z = 0.0;

                points.points.push_back(p);
            }
            angle += processed_scan_.angle_increment;
        }

        best_gap_points_pub_->publish(points);

        // Debugging: Log the number of points published
        RCLCPP_DEBUG(this->get_logger(), "Published %zu best gap points", points.points.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGap>());
    rclcpp::shutdown();
    return 0;
}
