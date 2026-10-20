#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include <sstream>
#include <string>
#include <cmath>

class ServoFixed : public rclcpp::Node
{
public:
    ServoFixed() : Node("servo_fixed")
    {
        // 串口设备名参数化，避免写死 /dev/ttyUSB0
        device_ = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");

        // 打开串口
        try
        {
            serial_port_.Open(device_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            RCLCPP_INFO(this->get_logger(), "Connected to %s", device_.c_str());
        }
        catch (const LibSerial::OpenFailed &)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s!", device_.c_str());
            return;
        }

        // 固定角度（可改）
        angleA_ = 90.0;
        angleB_ = 15.0;
        angleC_ = 165.0;
        angleD_ = 0.0;

        // 发送一次
        send_angles();

        // 每隔一段时间重复发送（防止舵机断信号复位）
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&ServoFixed::send_angles, this));

        RCLCPP_INFO(this->get_logger(), "Servo fixed-angle node started (A=%.1f, B=%.1f, C=%.1f, D=%.1f)",
                    angleA_, angleB_, angleC_, angleD_);
    }

private:
    void send_angles()
    {
        // 协议必须和 joystick_arm.cpp / joystick_gripper.cpp 保持一致：
        //   <A,B,C,D>\n
        // 原来这里发的是 "A=90 B=15 C=165 D=0"，与另外两个节点不兼容
        std::stringstream ss;
        ss << "<"
           << std::lround(angleA_) << ","
           << std::lround(angleB_) << ","
           << std::lround(angleC_) << ","
           << std::lround(angleD_) << ">\n";
        std::string cmd = ss.str();

        try
        {
            serial_port_.Write(cmd);
            RCLCPP_INFO(this->get_logger(), "Sent: %s", cmd.c_str());
        }
        catch (const LibSerial::NotOpen &)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open, cannot send!");
        }
    }

    LibSerial::SerialPort serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string device_;
    double angleA_, angleB_, angleC_, angleD_;
};

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoFixed>());
    rclcpp::shutdown();
    return 0;
}
