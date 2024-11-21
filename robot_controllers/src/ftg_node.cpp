// src/ftg_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include <algorithm>
#include <vector>
#include <cmath>
#include <chrono> // Include for working with system time

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
        this->declare_parameter<double>("speed_safety_radius", 0.15); 
        this->declare_parameter<double>("bubble_safety_radius", 0.05); 
        this->declare_parameter<double>("emergency_stop_fov_ratio", 0.2); 
        this->declare_parameter<double>("emergency_stop_distance", 0.1); 
        this->declare_parameter<double>("disparity_threshold", 0.5); 
        this->declare_parameter<double>("disparity_width_ratio_from_car_width", 0.5);  // percentage
        this->declare_parameter<int>("scan_filter_window_size", 5);  // scan filter window size
        this->declare_parameter<double>("reverse_speed", 0.2);
        this->declare_parameter<double>("reverse_time_period", 0.2);
        this->declare_parameter<double>("numer_of_laps_per_mission", 1.0);
        this->declare_parameter<double>("zone_entrance_time_period", 0.2);
        


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
        bubble_safety_radius_ = this->get_parameter("bubble_safety_radius").as_double();
        speed_safety_radius_ = this->get_parameter("speed_safety_radius").as_double();
        emergency_stop_fov_ratio_ = this->get_parameter("emergency_stop_fov_ratio").as_double();
        emergency_stop_distance_ = this->get_parameter("emergency_stop_distance").as_double();
        disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
        disparity_width_ratio_from_car_width_ = this->get_parameter("disparity_width_ratio_from_car_width").as_double();
        scan_filter_window_size_ = static_cast<size_t>(this->get_parameter("scan_filter_window_size").as_int());
        rev_time_period_ = this->get_parameter("reverse_time_period").as_double();
        rev_speed_ = this->get_parameter("reverse_speed").as_double();
        NUMBER_OF_LAPS_PER_MISSION = this->get_parameter("numer_of_laps_per_mission").as_double();
        zone_entrance_time_period_ = this->get_parameter("zone_entrance_time_period").as_double();
        

        // Initialize processed_scan_
        processed_scan_.header.frame_id = "lidar_frame"; // Update as per your frame
        processed_scan_.range_min = 0.0; // Will be set based on original scan
        processed_scan_.range_max = static_cast<float>(max_range_);
        // processed_scan_.angle_min, angle_max, angle_increment, and ranges will be set in preprocessLidar

        // Subscribers and Publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FollowTheGap::scanCallback, this, std::placeholders::_1));

        lap_count_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/lap_counter", 10, std::bind(&FollowTheGap::labCountCallback, this, std::placeholders::_1));

        lin_in_entrance_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/line_in_entrance", 10, std::bind(&FollowTheGap::lineEntranceCallback, this, std::placeholders::_1));
        
        state_machine_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/state_machine_command", 10, std::bind(&FollowTheGap::stateMachineCallback, this, std::placeholders::_1));
        

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

            // Publisher for best gap points
        lap_counting_reset_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/lap_counter_reset", 10);

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
    double speed_safety_radius_;
    double bubble_safety_radius_;
    double emergency_stop_fov_ratio_;
    double emergency_stop_distance_;
    double disparity_threshold_;
    double disparity_width_ratio_from_car_width_;
    size_t scan_filter_window_size_;
    double current_steering_angle_;

    // State of the state machine
    bool RUN_STATE_MACHINE = false;
    bool START_STATE = false;
    bool FTG_STATE = false;
    bool ZONE_ENTRANCE_STATE = false;
    bool WAIT_FOR_PARK_STATE = false;
    bool PARTK_STATE = false;
    bool END_STATE = false;
    // bool DRIVE_BACKWARD_STATE = false;

    // Variables for the driveBackward() function
    double rev_start_time_ = 0.0;
    double rev_current_time_ = 0.0;
    double rev_speed_ = 0.2; // TODO make a parameter
    double rev_time_period_ = 0.5; // seconds. TODO make a parameter

    // Zone entrance times
    double zone_current_time_  = 0.0;
    double zone_start_time_ = 0.0 ;
    double zone_entrance_time_period_ = 0.3;

    // lap counting variables
    bool is_line_in_entrance_ = false;
    double lap_count_ = 0.0;
    double NUMBER_OF_LAPS_PER_MISSION = 1.0; // TODO make a parameter

    // Processed LaserScan
    sensor_msgs::msg::LaserScan processed_scan_;
    // sensor_msgs::msg::LaserScan raw_scan_;

    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lap_count_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lin_in_entrance_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_machine_sub_;

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

    // Lap counting reset publisher
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lap_counting_reset_pub_;

    // Previous steering angle for smoothing
    float previous_steering_angle_;

    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        stateMachineLoop(scan_msg);
    }

    void labCountCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        lap_count_ = msg->data;
    }

    void lineEntranceCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_line_in_entrance_ = msg->data;
    }

    void stateMachineCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RUN_STATE_MACHINE = msg->data;
        if (RUN_STATE_MACHINE)
        {
            if( ! START_STATE )
            {
                resetStates();
                START_STATE = true;
            }

        }
    }

    void stateMachineLoop(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        
        if (! RUN_STATE_MACHINE)
        {
            RCLCPP_WARN(this->get_logger(), "State machine is not running. Publish to the /state_machine_command topic to start");
            publishDriveCommand(0.0, 0.0);
            resetStates();
            lap_count_ = 0;
            is_line_in_entrance_ = false;
            return;
        }
        
        if ( isEmergencyStop(scan_msg) )
        {
            if (publish_speed_) publishDriveCommand(0.0, 0.0);
            rev_start_time_ = getCurrentTimeInSeconds();
            driveBackward();
            return;
        }
        
        if (START_STATE)
        {
            executeStart();
        }

        else if (FTG_STATE)
        {
            executeFTG(scan_msg);
        }

        else if (ZONE_ENTRANCE_STATE)
        {
            executeZoneEntrance(scan_msg);
        }

        else if (WAIT_FOR_PARK_STATE)
        {
            executeWaitForPark();
        }

        else if (PARTK_STATE)
        {
            executePark();
        }

        else if (END_STATE)
        {
            executeEnd();
        }

        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unkown state!");    
        }

        return;
    }
    
    void resetStates(void)
    {
        START_STATE = false;
        FTG_STATE = false;
        ZONE_ENTRANCE_STATE = false;
        WAIT_FOR_PARK_STATE = false;
        PARTK_STATE = false;
        END_STATE = false;
    }

    void executeStart()
    {
        RCLCPP_INFO(this->get_logger(), "In START_STATE");
        resetStates();
        FTG_STATE = true;
        std_msgs::msg::Empty empty_msg;
        lap_counting_reset_pub_->publish(empty_msg);
        RCLCPP_INFO(this->get_logger(), "Going from START_STATE -> FTG_STATE");
    }
    
    

    void executeFTG(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        RCLCPP_INFO(this->get_logger(), "In FTG_STATE");
        if ( areLapsCompleted() )
        {
            resetStates();
            RCLCPP_INFO(this->get_logger(), "Going from FTG_STATE -> WAIT_FOR_PARK_STATE");
            zone_start_time_ = getCurrentTimeInSeconds();
            ZONE_ENTRANCE_STATE = true;

            return;
        }

        followTheGap(scan_msg);

        return;        
    }

    void executeZoneEntrance(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        
        RCLCPP_INFO(this->get_logger(), "In ZONE_ENTRANCE_STATE");
        
        if (!inZone()) // need to wait
        {
            followTheGap(scan_msg);
            return;
        }  
        
        // Otherwise, done, and proceed to next state
        resetStates();
        WAIT_FOR_PARK_STATE = true;
        RCLCPP_INFO(this->get_logger(), "Going from ZONE_ENTRANCE_STATE -> WAIT_FOR_PARK_STATE");
    }

    void executeWaitForPark()
    {
        RCLCPP_INFO(this->get_logger(), "In WAIT_FOR_PARK_STATE");
        // TODO Implement
        if ( isParkFound() )
        {
            resetStates();
            RCLCPP_INFO(this->get_logger(), "Found parking. Going from WAIT_FOR_PARK_STATE -> PARTK_STATE");
            PARTK_STATE = true;
        }
        else{
            resetStates();
            RCLCPP_INFO(this->get_logger(), "Did NOT find parking. Going from WAIT_FOR_PARK_STATE -> END_STATE");
            END_STATE = true;
        }

        return;
    }

    void executePark()
    {
        RCLCPP_INFO(this->get_logger(), "In PARK_STATE");
        // TODO Implement
        
        // WARNING Just going to END_STATE for now
        resetStates();
        RCLCPP_INFO(this->get_logger(), "Going from PARK_STATE -> END_STATE");
        END_STATE = true;
    }

    void executeEnd()
    {
        RCLCPP_INFO(this->get_logger(), "In the END_STATE");
        resetStates();
        RCLCPP_INFO(this->get_logger(), "Mission is completed. Stopping the state machine.");
        RUN_STATE_MACHINE = false;

        return;
    }

    
    void followTheGap(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
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
        gap = findDeepestSubWindow(gap.first, gap.second);
        gap = findMostContinuousGap(gap.first, gap.second);

        // Publish the best gap points
        publishBestGapPoints(gap.first, gap.second, scan_msg->header);

        // Step 5: Check if no valid gap is found
        if (gap.first == 0 && gap.second == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No valid gap found. Stopping the robot.");
            // Publish zero velocity command
            if (publish_speed_) publishDriveCommand(0.0f, 0.0f);

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
        // size_t best_point_idx = findBestPoint(gap.first, gap.second);
        size_t best_point_idx = findMidPointInGap(gap.first, gap.second);
        

        // Step 7: Compute steering angle and speed
        RCLCPP_DEBUG(this->get_logger(), "Step 7: Computing steering angle and speed");
        float steering_angle = calculateSteeringAngle(best_point_idx);
        float speed = computeSpeed();
        current_steering_angle_ = steering_angle;

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

        return;
    }

    bool isEmergencyStop(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::vector<float> ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());

        // Remove NaN and Inf values
        ranges.erase(std::remove_if(ranges.begin(), ranges.end(), [](float range) {
            return std::isnan(range) || std::isinf(range);
        }), ranges.end());

        int num_ranges = ranges.size();
        int front_angle = num_ranges / 2;
        int angle_range = emergency_stop_fov_ratio_* num_ranges ;  

        int start_idx = std::max(front_angle - angle_range, 0);
        int end_idx = std::min(front_angle + angle_range, num_ranges - 1);

        std::vector<float> front_ranges(ranges.begin() + start_idx, ranges.begin() + end_idx + 1);

        // Filter out any zero values to avoid false readings
        std::vector<float> valid_front_ranges;
        for (float range : front_ranges)
        {
            if (range > 0.0)
                valid_front_ranges.push_back(range);
        }

        float min_range = max_range_;
        if (!valid_front_ranges.empty())
        {
            min_range = *std::min_element(valid_front_ranges.begin(), valid_front_ranges.end());
        }

        if (min_range < emergency_stop_distance_)
        {
            RCLCPP_WARN(this->get_logger(), "Emergency stop triggered! Obstacle at %.2f meters.", min_range);
            return true;
        }
        return false;
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

        // Replace INF and NaN using a sliding window average
        size_t window_size = scan_filter_window_size_; // Define the size of the window for averaging
        float max_range = static_cast<float>(max_range_);

        for (size_t i = 0; i < limited_ranges.size(); ++i)
        {
            if (std::isnan(limited_ranges[i]) || std::isinf(limited_ranges[i]))
            {
                float sum = 0.0f;
                size_t valid_count = 0;

                // Calculate the average of valid values in the window
                size_t window_start = (i >= window_size) ? (i - window_size) : 0;
                size_t window_end = std::min(i + window_size, limited_ranges.size() - 1);

                for (size_t j = window_start; j <= (window_end-1); ++j)
                {
                    if ((std::abs(limited_ranges[j] - limited_ranges[j+1]) < 0.11) && j != i && limited_ranges[j] > 0.0f && limited_ranges[j] <= max_range && !std::isnan(limited_ranges[j]) && !std::isinf(limited_ranges[j]))
                    {
                        sum += limited_ranges[j];
                        valid_count++;
                    }
                }

                // Replace with the average if valid points are found, otherwise set to max_range
                if (valid_count > 0)
                {
                    limited_ranges[i] = sum / static_cast<float>(valid_count);
                }
                else
                {
                    limited_ranges[i] = max_range; // Default to max_range if no valid values are found
                }

                if (limited_ranges[i] > max_range) limited_ranges[i] = max_range; // clipping
            }
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

            // Check if intensity is 1 or 2 or 3
            // label == 1 -> green, label == 2 --> parking color(pink), label == 3 --> red
            //int int_intensity = static_cast<int>(intensity); // From float to int

            if ((intensity == 2.0 || intensity == 4.0 || intensity == 5.0) )
            {
                if (range < min_range)
                {
                    min_range = range;
                    closest_idx = i;
                    label = intensity;
                    found = true;
                }
            }
        }

        if (found)
        {
            RCLCPP_INFO(this->get_logger(), "Closest labeled object at index %zu, range %.2f, label %d", closest_idx, min_range, label);

            // Modify the ranges based on the label
            // label == 1 -> green, label == 2 --> parking color(pink), label == 3 --> red
            if (label == 2.0)
            {
                // Green object: modify ranges on the right
                for (size_t i = 0; i < closest_idx; ++i)
                {
                    processed_scan_.ranges[i] = 0.0; //min_range;
                }
            }
            else if (label == 5.0)
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
            RCLCPP_DEBUG(this->get_logger(), "No valid labeled objects found in intensities.");
        }
    }

    void applySafetyBubble()
    {
        // Find the index of the closest point
        size_t closest_idx = 0;
        float min_range = std::numeric_limits<float>::max();

        for (size_t i = 0; i < processed_scan_.ranges.size(); ++i)
        {
            float range = processed_scan_.ranges[i];
            if (range > 0.0f && range < min_range)
            {
                min_range = range;
                closest_idx = i;
            }
        }

        // Prevent out-of-bounds
        if (closest_idx >= processed_scan_.ranges.size())
        {
            RCLCPP_WARN(this->get_logger(), "Closest index out of bounds. Skipping safety bubble.");
            return;
        }

        // Calculate theta using arcsine instead of atan2
        float ratio = bubble_safety_radius_ / min_range;

        // Clamp the ratio to the valid range for asin [-1, 1]
        if (ratio > 1.0f)
        {
            ratio = 1.0f;
            RCLCPP_WARN(this->get_logger(),
                        "Ratio (bubble_safety_radius_ / min_range) exceeds 1. Clamping to 1. Setting theta to pi/2 radians.");
        }
        else if (ratio < -1.0f)
        {
            ratio = -1.0f;
            RCLCPP_WARN(this->get_logger(),
                        "Negative ratio encountered. Clamping to -1. Setting theta to -pi/2 radians.");
        }

        float theta = std::asin(ratio); // Using arcsine instead of atan2

        // Calculate the number of points to clear around the closest point
        size_t bubble_size = 0;
        if (processed_scan_.angle_increment > 0.0f)
        {
            bubble_size = static_cast<size_t>(theta / processed_scan_.angle_increment);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Angle increment is zero or negative. Cannot compute bubble size.");
            return; // Skip this iteration if angle_increment is invalid
        }

        // Ensure bubble_size is non-negative
        if (theta < 0.0f)
        {
            bubble_size = 0;
        }

        size_t start_idx = (closest_idx >= bubble_size) ? (closest_idx - bubble_size) : 0;
        size_t end_idx = std::min(closest_idx + bubble_size, processed_scan_.ranges.size() - 1);

        // Clear the ranges within the safety bubble by setting them to 0.0f
        std::fill(processed_scan_.ranges.begin() + start_idx,
                processed_scan_.ranges.begin() + end_idx + 1,
                0.0f);

        // Log the computed values
        RCLCPP_INFO(this->get_logger(), "Safety Bubble Size: %zu points", bubble_size);
        RCLCPP_INFO(this->get_logger(), "Closest Index: %zu", closest_idx);
        RCLCPP_INFO(this->get_logger(), "Start Index: %zu", start_idx);
        RCLCPP_INFO(this->get_logger(), "End Index: %zu", end_idx);
        RCLCPP_INFO(this->get_logger(), "Safety Bubble Theta: %.4f radians (%.2f degrees)", theta, theta * 180.0 / M_PI);
    }


    void applyDisparityExtender()
    {
        size_t num_ranges = processed_scan_.ranges.size();
        float angle_increment = processed_scan_.angle_increment;
        float car_half_width = static_cast<float>(car_width_ / 2.0);
        float car_width_portion = disparity_width_ratio_from_car_width_ * static_cast<float>(car_width_);

        for (size_t i = 0; i < num_ranges - 1; ++i)
        {
            float r1 = processed_scan_.ranges[i];
            float r2 = processed_scan_.ranges[i + 1];

            // Skip if any of the ranges are zero (no obstacle)
            if (r1 == 0.0f || r2 == 0.0f)
                continue;

            float range_diff = std::abs(r2 - r1);

            if (range_diff > disparity_threshold_)
            {
                // Calculate the number of points to extend
                float min_range = std::min(r1, r2);

                // Prevent division by zero and ensure the argument for asin is within [-1, 1]
                float ratio = car_width_portion / min_range;
                if (ratio > 1.0f)
                {
                    // If the ratio exceeds 1, set theta to 90 degrees (pi/2 radians)
                    ratio = 1.0f;
                    RCLCPP_WARN(this->get_logger(),
                                "Ratio (car_width_portion / min_range) exceeds 1. Setting theta to pi/2 radians.");
                }
                else if (ratio < -1.0f)
                {
                    // Although unlikely, handle negative ratios just in case
                    ratio = -1.0f;
                    RCLCPP_WARN(this->get_logger(),
                                "Negative ratio encountered. Setting theta to -pi/2 radians.");
                }

                float theta = std::asin(ratio); // Using arcsine instead of atan2

                // Calculate the number of points to extend based on theta
                size_t num_points = 0;
                if (angle_increment > 0.0f)
                {
                    num_points = static_cast<size_t>(theta / angle_increment);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Angle increment is zero or negative.");
                    continue; // Skip this iteration if angle_increment is invalid
                }

                if (r1 < r2)
                {
                    // Extend to the left
                    size_t end_idx = std::min(i + num_points, num_ranges - 1);
                    std::fill(processed_scan_.ranges.begin() + i + 1,
                            processed_scan_.ranges.begin() + end_idx + 1,
                            r1);

                    // Debugging: Log disparity extender actions
                    RCLCPP_INFO(this->get_logger(),
                                "Applied disparity extender at index %zu with theta %.4f degrees and num_points %zu, car_width_portion %.3f, min_range %0.3f",
                                i, theta*180.0/3.1415, num_points, car_width_portion, min_range);
                    i = i + num_points;
                }
                else
                {
                    // Extend to the right
                    size_t start_idx = (i >= num_points) ? (i - num_points + 1) : 0;
                    std::fill(processed_scan_.ranges.begin() + start_idx,
                            processed_scan_.ranges.begin() + i + 1,
                            r2);
                    // Debugging: Log disparity extender actions
                    RCLCPP_INFO(this->get_logger(),
                                "Applied disparity extender at index %zu with theta %.4f degrees and num_points %zu, car_width_portion %.3f, min_range %0.3f",
                                i, theta*180.0/3.1415, num_points, car_width_portion, min_range);
                }

                
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

 
    /**
    * @brief Finds the most continuous sub-gap within a given gap based on a discontinuity threshold.
    *
    * @param gap_start The starting index of the deepest gap.
    * @param gap_end The ending index of the deepest gap.
    * @return A pair containing the start and end indices of the most continuous sub-gap.
    */
    std::pair<size_t, size_t> findMostContinuousGap(size_t gap_start, size_t gap_end)
    {
        const auto& ranges = processed_scan_.ranges;
        double threshold = discontinuity_threshold_;

        size_t best_start = gap_start;
        size_t best_end = gap_start;
        size_t current_start = gap_start;
        size_t current_end = gap_start;

        for (size_t i = gap_start; i < gap_end; ++i)
        {
            // Calculate the absolute difference between consecutive range points
            if (std::abs(ranges[i + 1] - ranges[i]) <= threshold)
            {
                // Continue the current continuous gap
                current_end = i + 1;
            }
            else
            {
                // Check if the current continuous gap is the longest so far
                if ((current_end - current_start) > (best_end - best_start))
                {
                    best_start = current_start;
                    best_end = current_end;
                }
                // Start a new continuous gap
                current_start = i + 1;
                current_end = i + 1;
            }
        }

        // After the loop, check the last continuous gap
        if ((current_end - current_start) > (best_end - best_start))
        {
            best_start = current_start;
            best_end = current_end;
        }

        // Handle the case where no continuous sub-gap is found
        if (best_end < best_start)
        {
            best_start = gap_start;
            best_end = gap_start;
        }

        RCLCPP_DEBUG(this->get_logger(), "Most continuous gap from index %zu to %zu", best_start, best_end);

        return {best_start, best_end};
    }


    size_t findMidPointInGap(size_t start_idx, size_t end_idx)
    {
        
        // Select mid-point in the gap
        const auto& ranges = processed_scan_.ranges;
        size_t gap_size = end_idx - start_idx;

        if (gap_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Gap size is zero. Selecting start index as best point.");
            return start_idx;
        }


        size_t best_point_idx = (start_idx + end_idx) / 2;

        RCLCPP_DEBUG(this->get_logger(), "Best point in gap at index %zu with range %.2f meters",
                     best_point_idx, ranges[best_point_idx]);

        return best_point_idx;
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

    // size_t findBestPoint(size_t start_idx, size_t end_idx)
    // {
    //     const auto& ranges = processed_scan_.ranges;

    //     if (start_idx >= end_idx || end_idx >= ranges.size())
    //     {
    //         RCLCPP_WARN(this->get_logger(), "Invalid index range. Returning start index.");
    //         return start_idx;
    //     }

    //     // Initialize max depth and index
    //     float max_depth = ranges[start_idx];
    //     size_t best_point_idx = start_idx;

    //     // Iterate through the range to find the maximum depth
    //     for (size_t i = start_idx; i <= end_idx; ++i)
    //     {
    //         if (ranges[i] > max_depth)
    //         {
    //             max_depth = ranges[i];
    //             best_point_idx = i;
    //         }
    //     }

    //     RCLCPP_DEBUG(this->get_logger(), "Best point in gap at index %zu with range %.2f meters",
    //                 best_point_idx, max_depth);

    //     return best_point_idx;
    // }

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

        if (min_range > (speed_safety_radius_*2))
        {
            speed = static_cast<float>(min_speed_ + (max_speed_ - min_speed_) * ((min_range - speed_safety_radius_) / (max_range_ - speed_safety_radius_)));
            speed = std::min(speed, static_cast<float>(max_speed_));
        }
        // else
        // {
        //     speed = 0.0f; // Stop if obstacle is too close
        // }

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
    
    
    // Function to get the current time in decimal seconds
    double getCurrentTimeInSeconds()
    {
        return std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    // Reverse function
    // published fixed (low) speed and 0 steering angle to allow the robot to recover from an emergency stop
    void driveBackward(void)
    {
        rev_current_time_ = getCurrentTimeInSeconds();
        rev_start_time_ = getCurrentTimeInSeconds();
        RCLCPP_INFO(this->get_logger(), "Driving backward");
        while( (rev_current_time_ - rev_start_time_) < rev_time_period_ )
        {
            publishDriveCommand(-current_steering_angle_, -rev_speed_);
            rev_current_time_ = getCurrentTimeInSeconds();
        }
        RCLCPP_INFO(this->get_logger(), "DONE driving backward");
        publishDriveCommand(0.0, 0.0);
        return;
    }
    
    bool areLapsCompleted(void)
    {
        if (lap_count_ == NUMBER_OF_LAPS_PER_MISSION && is_line_in_entrance_) return true;
        return false;
    }
    bool inZone()
    {
        zone_current_time_ = getCurrentTimeInSeconds();
        if (zone_current_time_ - zone_start_time_ < zone_entrance_time_period_ )
        {
            return false;
        }
        return true;
    }

    bool isParkFound()
    {
        // TODO Implement
        return false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGap>());
    rclcpp::shutdown();
    return 0;
}
