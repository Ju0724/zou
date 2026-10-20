#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <libserial/SerialPort.h>
#include <string>
#include <sstream>
#include <mutex>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class GazeboToServoWithGripper : public rclcpp::Node
{
public:
    GazeboToServoWithGripper()
        : Node("gazebo_to_servo_with_gripper"),
          baseA0_(90.0), baseB0_(15.0), baseC0_(165.0),
          s_(0.0), angleD_(0.0), initialized_(false)
    {
        // === 串口初始化 ===
        device_ = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
        try {
            serial_.Open(device_);
            serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            RCLCPP_INFO(this->get_logger(), "Connected to %s", device_.c_str());
        } catch (const LibSerial::OpenFailed &) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s!", device_.c_str());
            return;
        }

        // === 订阅 Gazebo joint_states ===
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&GazeboToServoWithGripper::joint_callback, this, _1));

        // === 订阅手柄控制夹爪 ===
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", qos, std::bind(&GazeboToServoWithGripper::joy_callback, this, _1));

        // === 定时器（防超时）===
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GazeboToServoWithGripper::send_angles, this));

        RCLCPP_INFO(this->get_logger(), "🎮 Gazebo sync + Joy gripper control initialized.");
    }

private:
    // === joint_states 回调 ===
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 3)
            return;

        // === 初始化时记录起点 ===
        if (!initialized_) {
            init_joint_pos_ = msg->position;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Received initial joint state, using it as reference zero.");
            return;
        }

        // === 当前关节变化量（弧度差）===
        double deltaA = msg->position[0] - init_joint_pos_[0];
        double deltaB = msg->position[1] - init_joint_pos_[1];
        double deltaC = msg->position[2] - init_joint_pos_[2];

        // === 弧度 → 角度 ===
        deltaA *= 180.0 / M_PI;
        deltaB *= 180.0 / M_PI;
        deltaC *= 180.0 / M_PI;

        // === A 舵机角 ===
        servoA_ = baseA0_ - deltaA * (72.0 / 28.0); // 减速比映射
        servoA_ = std::clamp(servoA_, 0.0, 180.0);

        // === B 舵机角 ===
        servoB_ = baseB0_ - deltaB; // 同向变化
        servoB_ = std::clamp(servoB_, 0.0, 80.0);

        // === C 舵机角（关键修正部分）===
        // Gazebo joint 是相对变化量 → joint_C = 83.5° + ΔC
        double B_joint = baseB0_ - deltaB;   // 当前关节 B（°）
        double C_joint = 83.5 + deltaC;      // 当前关节 C（°）（初始时 C_joint=83.5）
        servoC_ = C_joint - B_joint + 83.5;  // 反推舵机角
        servoC_ = std::clamp(servoC_, 0.0, 180.0);

        // === 夹爪角 D 由手柄控制 ===
        send_angles();

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "ΔA=%.2f ΔB=%.2f ΔC=%.2f | Joint[B=%.2f C=%.2f] | Servo[A=%.1f B=%.1f C=%.1f]",
            deltaA, deltaB, deltaC, B_joint, C_joint, servoA_, servoB_, servoC_);
    }

    // === Joy 回调：控制夹爪 D ===
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < 3) return;

        double hall_axis = msg->axes[2];
        double input_min = -0.61;
        double input_max = 0.62;
        double output_min = 0.0;
        double output_max = 30.0;

        hall_axis = std::clamp(hall_axis, input_min, input_max);
        double ratio = (hall_axis - input_min) / (input_max - input_min);
        angleD_ = output_max - ratio * (output_max - output_min);

        send_angles();
    }

    // === 串口发送 ===
    void send_angles()
    {
        if (!initialized_) return;

        std::stringstream ss;
        ss << "<"
           << std::lround(servoA_) << ","
           << std::lround(servoB_) << ","
           << std::lround(servoC_) << ","
           << std::lround(angleD_) << ">\n";
        std::string cmd = ss.str();

        // 直接同步写：原来每条 joint_states 都 std::async 起一个线程且不等待，
        // 100Hz 下会持续创建线程，属于明显的资源泄漏
        try {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            serial_.Write(cmd);
            serial_.DrainWriteBuffer();
        } catch (const LibSerial::NotOpen &) {
            RCLCPP_ERROR(this->get_logger(), "Serial not open!");
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Servo cmd: A=%.1f B=%.1f C=%.1f D=%.1f",
                             servoA_, servoB_, servoC_, angleD_);
    }

    // === 成员变量 ===
    LibSerial::SerialPort serial_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex serial_mutex_;

    // 初始舵机角度
    std::string device_;
    double baseA0_, baseB0_, baseC0_;
    double s_;
    double angleD_;

    // Gazebo 起始角
    std::vector<double> init_joint_pos_;
    bool initialized_;

    // 实时舵机角
    double servoA_{NAN}, servoB_{NAN}, servoC_{NAN};
};

// ---- 主函数 ----
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboToServoWithGripper>());
    rclcpp::shutdown();
    return 0;
}
