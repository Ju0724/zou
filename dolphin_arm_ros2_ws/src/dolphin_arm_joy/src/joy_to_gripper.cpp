#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <vector>

class JoyToGripper : public rclcpp::Node
{
public:
    JoyToGripper()
        : Node("joy_to_gripper")
    {
        // 🎮 手柄输入订阅
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToGripper::joy_callback, this, std::placeholders::_1));

        // 🦾 夹爪发布器
        gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/gripper_controller/commands", 10);

        // 定时发布（20Hz）
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JoyToGripper::timer_callback, this));

        // === 参数初始化 ===
        grip_pos_ = -0.017;  // 初始为闭合位置
        // 与 dolphin_arm_bridge/joystick_gripper.cpp 保持同一标定范围，
        // 原来写 -0.43，同一条霍尔轴在两个节点里范围不一致
        input_min_ = -0.61;  // 霍尔最小值（张开）
        input_max_ =  0.62;  // 霍尔最大值（闭合）
        output_min_ = -0.017;
        output_max_ = 0.8;
        neutral_ = 0.0;
        last_hall_ = 0.0;
        calibrated_ = false;
        axis_id_ = 2;  // 默认霍尔对应 axes[2]

        RCLCPP_INFO(this->get_logger(),
                    "✅ Gripper controller initialized (axis=%d, range=[%.3f, %.3f])",
                    axis_id_, input_min_, input_max_);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= static_cast<size_t>(axis_id_))
            return;


        double hall_value = msg->axes[axis_id_];

        // 第一次读取时自动记录中立点
        if (!calibrated_)
        {
            neutral_ = hall_value;
            // 同时初始化 last_hall_，否则校准后的第一个控制周期会读到未初始化内存
            last_hall_ = hall_value;
            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "📟 Neutral calibrated at %.3f", neutral_);
            return;
        }

        // 保存霍尔原始值
        last_hall_ = hall_value;
    }

    void timer_callback()
    {
        if (!calibrated_) return;

        // === 霍尔线性映射 ===
        double hall_clamped = std::clamp(last_hall_, input_min_, input_max_);
        double ratio = (hall_clamped - input_min_) / (input_max_ - input_min_);
        grip_pos_ = output_max_ - ratio * (output_max_ - output_min_);
        grip_pos_ = std::clamp(grip_pos_, output_min_, output_max_);

        // === 发布到 gripper_controller ===
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {grip_pos_};
        gripper_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "📟 Hall=%.3f | Grip=%.3f", last_hall_, grip_pos_);
    }

    // ROS 通信
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 控制变量
    double grip_pos_;
    double last_hall_;
    double input_min_, input_max_;
    double output_min_, output_max_;
    double neutral_;
    bool calibrated_;
    int axis_id_;
};

// ---- 主函数 ----
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToGripper>());
    rclcpp::shutdown();
    return 0;
}
