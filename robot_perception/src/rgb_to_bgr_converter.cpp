// rgb_to_bgr_converter.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RGBToBGRNode : public rclcpp::Node
{
public:
    RGBToBGRNode()
    : Node("rgb_to_bgr_converter")
    {
        // Create the subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/input_image",
            10,
            std::bind(&RGBToBGRNode::image_callback, this, std::placeholders::_1)
        );

        // Create the publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/output_image",
            10
        );

        RCLCPP_INFO(this->get_logger(), "RGB to BGR Converter Node has been started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding == "rgb8")
        {
            // Convert the ROS Image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            }
            catch (cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // Convert RGB to BGR
            cv::Mat bgr_image;
            cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);

            // Convert back to ROS Image message
            auto bgr_msg = cv_bridge::CvImage(
                msg->header,  // Preserve the original header
                "bgr8",       // Set the encoding to 'bgr8'
                bgr_image     // The converted image
            ).toImageMsg();

            bgr_msg->header.frame_id = "optical_camera_link";  // Set the frame_id

            // Publish the converted image
            publisher_->publish(*bgr_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Image encoding is not rgb8, skipping conversion.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBToBGRNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
