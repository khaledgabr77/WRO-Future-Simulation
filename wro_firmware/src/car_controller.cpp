#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <libserial/SerialPort.h>
#include <mutex>
#include <cstdio>


// Helper function to convert integer to LibSerial::BaudRate
LibSerial::BaudRate getBaudRate(int baud_rate)
{
    static const std::unordered_map<int, LibSerial::BaudRate> baud_rate_map = {
        {50, LibSerial::BaudRate::BAUD_50}, {75, LibSerial::BaudRate::BAUD_75},
        {110, LibSerial::BaudRate::BAUD_110}, {134, LibSerial::BaudRate::BAUD_134},
        {150, LibSerial::BaudRate::BAUD_150}, {200, LibSerial::BaudRate::BAUD_200},
        {300, LibSerial::BaudRate::BAUD_300}, {600, LibSerial::BaudRate::BAUD_600},
        {1200, LibSerial::BaudRate::BAUD_1200}, {1800, LibSerial::BaudRate::BAUD_1800},
        {2400, LibSerial::BaudRate::BAUD_2400}, {4800, LibSerial::BaudRate::BAUD_4800},
        {9600, LibSerial::BaudRate::BAUD_9600}, {19200, LibSerial::BaudRate::BAUD_19200},
        {38400, LibSerial::BaudRate::BAUD_38400}, {57600, LibSerial::BaudRate::BAUD_57600},
        {115200, LibSerial::BaudRate::BAUD_115200}, {230400, LibSerial::BaudRate::BAUD_230400},
#ifdef __linux__
        {460800, LibSerial::BaudRate::BAUD_460800}, {500000, LibSerial::BaudRate::BAUD_500000},
        {576000, LibSerial::BaudRate::BAUD_576000}, {921600, LibSerial::BaudRate::BAUD_921600},
        {1000000, LibSerial::BaudRate::BAUD_1000000}, {1152000, LibSerial::BaudRate::BAUD_1152000},
        {1500000, LibSerial::BaudRate::BAUD_1500000}, {2000000, LibSerial::BaudRate::BAUD_2000000},
        {2500000, LibSerial::BaudRate::BAUD_2500000}, {3000000, LibSerial::BaudRate::BAUD_3000000},
        {3500000, LibSerial::BaudRate::BAUD_3500000}, {4000000, LibSerial::BaudRate::BAUD_4000000},
#endif
    };

    if (baud_rate_map.find(baud_rate) != baud_rate_map.end())
    {
        return baud_rate_map.at(baud_rate);
    }
    else
    {
        throw std::invalid_argument("Invalid baud rate");
    }
}

class CarController : public rclcpp::Node
{
public:
    CarController();
    ~CarController();

private:
    // Callback functions
    void drive_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void steer_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // Methods to send commands over serial
    void send_velocity_command();
    void send_angle_command();

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drive_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_subscription_;

    // Variables
    double velocity_;
    double angle_;

    // Serial port
    LibSerial::SerialPort serial_port_;

    // Mutex for thread-safe serial communication
    std::mutex serial_mutex_;

    // Serial port parameters
    std::string port_;
    int baud_rate_;
};

CarController::CarController() : Node("car_controller"), velocity_(0.0), angle_(0.0)
{
    // Declare and get parameters
    this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
    this->declare_parameter<int>("baud_rate", 9600);
    port_ = this->get_parameter("port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // Initialize serial communication
    try
    {
        serial_port_.Open(port_);
        serial_port_.SetBaudRate(getBaudRate(baud_rate_));
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        // serial_port_.SetTimeout(LibSerial::Seconds(1));

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
    }
    catch (const LibSerial::OpenFailed&)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        rclcpp::shutdown();
        return;
    }
    catch (const LibSerial::AlreadyOpen&)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port is already open.");
        rclcpp::shutdown();
        return;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // Create subscriptions
    drive_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/drive", 10, std::bind(&CarController::drive_callback, this, std::placeholders::_1));

    steer_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "/steer_angle", 10, std::bind(&CarController::steer_callback, this, std::placeholders::_1));
}

CarController::~CarController()
{
    if (serial_port_.IsOpen())
    {
        serial_port_.Close();
        RCLCPP_INFO(this->get_logger(), "Serial port closed.");
    }
}

void CarController::drive_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    velocity_ = msg->data;
    send_velocity_command();
}

void CarController::steer_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    angle_ = msg->data;
    send_angle_command();
}

void CarController::send_velocity_command()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    // Construct command string
    char command[50];
    snprintf(command, sizeof(command), "v%.4f,", velocity_);

    // Send command over serial
    try
    {
        serial_port_.Write(command);
        RCLCPP_DEBUG(this->get_logger(), "Sent velocity command: %s", command);
    }
    catch (const LibSerial::NotOpen&)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
    }
}

void CarController::send_angle_command()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    // Construct command string
    char command[50];
    snprintf(command, sizeof(command), "a%.4f,", angle_);

    // Send command over serial
    try
    {
        serial_port_.Write(command);
        RCLCPP_DEBUG(this->get_logger(), "Sent angle command: %s", command);
    }
    catch (const LibSerial::NotOpen&)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error writing to serial port: %s", e.what());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto car_controller = std::make_shared<CarController>();

    try
    {
        rclcpp::spin(car_controller);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(car_controller->get_logger(), "Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
