#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include <fstream>
#include <iomanip>


// ===== 工具函数 =====
inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

inline double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

// ===== 正运动学：由几何角 A,B,C（deg） -> 末端 xyz（mm）=====
// 注意：这里的 A,B,C 是几何推导里的“几何角”，不是 Gazebo 关节角。
void alg_positive_operation(double a_deg, double b_deg, double c_deg,
                            double* x_mm, double* y_mm, double* z_mm)
{
  double A = a_deg;
  double B = b_deg;
  double C = c_deg;

  // 由 As = A * (72/28)， (As-90)/(72/28) = A - 35
  double theta_deg = A - 35.0;
  double theta_rad = deg2rad(theta_deg);

  // Cs - 75.5 = (C - B + 96.5) - 75.5 = C - B + 21
  double Cs_minus_75_5_deg = C - B + 21.0;

  double inner =
      135.0 * std::cos(deg2rad(111.0 - B)) +
      145.0 * std::sin(deg2rad(Cs_minus_75_5_deg)) +
      67.0;

  *x_mm = std::cos(theta_rad) * inner;
  *y_mm = 65.5
        + 135.0 * std::sin(deg2rad(111.0 - B))
        - 145.0 * std::cos(deg2rad(Cs_minus_75_5_deg));
  *z_mm = std::sin(theta_rad) * inner;
}

// ===== 角度合法性检查 =====
bool check_angle(double A_deg, double B_deg, double C_deg)
{
  const double eps = 1.0;

  // A：大致在 0~180
  if (A_deg < -eps || A_deg > 180.0 + eps)
    return false;

  // B：允许到 -5°，避免轻微负数（数值误差）被判越界
  const double B_min = -5.0;
  const double B_max = 85.0;
  if (B_deg < B_min || B_deg > B_max + eps)
    return false;

  // C 的联动限制： (140° - B) ≤ C ≤ min(180°, 196° - B)
  double minC = 140.0 - B_deg;
  double maxC = (196.0 - B_deg < 180.0) ? (196.0 - B_deg) : 180.0;

  if (C_deg < minC - eps || C_deg > maxC + eps)
    return false;

  return true;
}

// ===== 逆运动学：由末端 xyz（mm） -> 几何角 A,B,C（deg）=====
// 这是标准两连杆（135mm / 145mm）平面解析解。
// 原实现里 temp2 的余弦用错了三角形顶点（分子写成 145²+L²-135²），
// 只有在 B=0 的奇异位姿附近才碰巧成立，B 不为 0 时解出的关节角无法反算回原位置。
// 这里遍历 "朝前/朝后" × "肘部两个分支"，返回离 seed 最近且满足限位的一组解。
bool inverse_operation(double x_mm, double y_mm, double z_mm,
                       double seedA_deg, double seedB_deg, double seedC_deg,
                       double* outA_deg, double* outB_deg, double* outC_deg)
{
  const double R = std::sqrt(x_mm * x_mm + z_mm * z_mm);
  if (R < 1e-6)
    return false;

  bool   found = false;
  double best_cost = std::numeric_limits<double>::max();

  for (int sign = 1; sign >= -1; sign -= 2)
  {
    const double inner = sign * R;                       // 平面内的径向长度（带符号）
    const double theta = std::atan2(z_mm / inner, x_mm / inner);

    const double U = inner - 67.0;                       // 去掉 7 + 60 的水平偏置
    const double V = y_mm - 65.5;
    const double L = std::hypot(U, V);

    if (L > 135.0 + 145.0 + 1e-6)                        // 超出可达范围
      continue;
    if (L < std::fabs(145.0 - 135.0) - 1e-6)
      continue;

    const double phi = std::atan2(V, U);
    const double cos_d = clamp(
        (135.0 * 135.0 + L * L - 145.0 * 145.0) / (2.0 * 135.0 * L), -1.0, 1.0);
    const double d = std::acos(cos_d);

    for (int elbow = 1; elbow >= -1; elbow -= 2)
    {
      const double p = phi + elbow * d;
      const double r = std::atan2(V - 135.0 * std::sin(p),
                                  U - 135.0 * std::cos(p));

      const double a = std::fmod(rad2deg(theta) + 35.0 + 360.0, 360.0);
      const double b = 111.0 - rad2deg(p);
      const double c = rad2deg(r) + b + 69.0;

      if (a < 0.0 || a > 180.0)
        continue;
      if (!check_angle(a, b, c))
        continue;

      const double cost = std::pow(a - seedA_deg, 2)
                        + std::pow(b - seedB_deg, 2)
                        + std::pow(c - seedC_deg, 2);
      if (cost < best_cost)
      {
        best_cost = cost;
        *outA_deg = a; *outB_deg = b; *outC_deg = c;
        found = true;
      }
    }
  }
  return found;
}

// =============== 主节点：末端固定速度 + IK ===============
class JoyToVelocity : public rclcpp::Node
{
public:
  JoyToVelocity()
  : Node("joy_to_velocity")
  {
    // 关节名需要和 URDF/controller 对齐
    joint_names_ = {
      "Fixed_Base_rot_1",
      "Rotary_Base_rot_2",
      "BC_Connecting_Rod_rot_3"
    };

    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    // ★ 末端位置发布器 /ee_position（单位 m）
    ee_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/ee_position", 10);

    // ★ 新增：日志文件路径参数（默认当前目录 ik_log.csv）
    log_path_ = this->declare_parameter<std::string>("log_path", "ik_log.csv");

    // 初始几何角（deg），对应 As=90, Bs=0, Cs=180 姿态
    A_deg_ = this->declare_parameter<double>("init_A_deg", 90.0);
    B_deg_ = this->declare_parameter<double>("init_B_deg", 0.0);
    C_deg_ = this->declare_parameter<double>("init_C_deg", 180.0);

    // 用 FK 算出初始末端位置（mm）
    alg_positive_operation(A_deg_, B_deg_, C_deg_, &x_mm_, &y_mm_, &z_mm_);

    // 末端速度（mm/s）
    vx_mm_s_ = this->declare_parameter<double>("vx_mm_s", 0.0);
    vy_mm_s_ = this->declare_parameter<double>("vy_mm_s", 0.0);
    vz_mm_s_ = this->declare_parameter<double>("vz_mm_s", -10.0);

    // 控制周期 dt（s）
    dt_ = this->declare_parameter<double>("dt", 0.02);  // 50Hz

    // J1(base) 的 Gazebo 关节限位（±35°）
    j1_min_rad_ = this->declare_parameter<double>("j1_min_rad", -deg2rad(35.0));
    j1_max_rad_ = this->declare_parameter<double>("j1_max_rad",  deg2rad(35.0));

    // ★ 新增：打开 CSV 日志（记录 IK 的输入/输出/命令）
    ik_log_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!ik_log_.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open log file: %s (logging disabled)", log_path_.c_str());
      log_enabled_ = false;
    } else {
      log_enabled_ = true;
      // 写表头
      ik_log_ << "t_s,"
              << "x_ref_mm,y_ref_mm,z_ref_mm,"
              << "A_deg,B_deg,C_deg,"
              << "j1_cmd_rad,j2_cmd_rad,j3_cmd_rad\n";
      ik_log_ << std::fixed << std::setprecision(6);
      t0_ = this->now();
      RCLCPP_INFO(this->get_logger(), "IK log enabled: %s", log_path_.c_str());
    }

    RCLCPP_INFO(this->get_logger(),
                "Cartesian vel IK: init xyz = [%.1f, %.1f, %.1f] mm, "
                "v = [%.1f, %.1f, %.1f] mm/s, dt=%.3f, J1_lim=[%.3f, %.3f]rad",
                x_mm_, y_mm_, z_mm_,
                vx_mm_s_, vy_mm_s_, vz_mm_s_, dt_,
                j1_min_rad_, j1_max_rad_);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&JoyToVelocity::timer_callback, this));
  }

  ~JoyToVelocity() override
  {
    if (ik_log_.is_open()) {
      ik_log_.flush();
      ik_log_.close();
    }
  }

private:
  void timer_callback()
  {
    // 1) 在任务空间积分末端位置（mm） —— 这就是 reference (x_ref,y_ref,z_ref)
    double target_x = x_mm_ + vx_mm_s_ * dt_;
    double target_y = y_mm_ + vy_mm_s_ * dt_;
    double target_z = z_mm_ + vz_mm_s_ * dt_;

    // 2) 用 IK 解出几何角 A,B,C（deg）
    double A_new, B_new, C_new;
    if (!inverse_operation(target_x, target_y, target_z,
                           A_deg_, B_deg_, C_deg_,
                           &A_new, &B_new, &C_new))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "IK failed at xyz=[%.1f, %.1f, %.1f]mm",
                           target_x, target_y, target_z);
      return;
    }

    if (!check_angle(A_new, B_new, C_new))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "IK angles out of range: A=%.2f, B=%.2f, C=%.2f deg at xyz=[%.1f,%.1f,%.1f]mm",
                           A_new, B_new, C_new, target_x, target_y, target_z);
      return;
    }

    // 3) 几何角 → Gazebo 关节角（rad）
    double j1_rad = deg2rad(A_new - 90.0);
    double j2_rad = deg2rad(B_new - 0.0);
    double j3_rad = deg2rad(C_new - 180.0);

    // 对 J1 应用 ±35° 限位
    j1_rad = clamp(j1_rad, j1_min_rad_, j1_max_rad_);

    // ★ 新增：写入 IK 日志（reference + IK 输出 + 下发命令）
    if (log_enabled_) {
      double t_s = (this->now() - t0_).seconds();
      ik_log_ << t_s << ","
              << target_x << "," << target_y << "," << target_z << ","
              << A_new << "," << B_new << "," << C_new << ","
              << j1_rad << "," << j2_rad << "," << j3_rad << "\n";
      // 保险：避免崩溃丢数据（科研实验建议保留）
      ik_log_.flush();
    }

    // 更新内部参考位置：用“真正下发的关节角”做 FK 回填，
    // 而不是直接等于 target_*。否则 J1 被 ±35° 截断后，
    // 参考点仍按未截断的方向继续积分，误差会单向累积。
    A_deg_ = rad2deg(j1_rad) + 90.0;
    B_deg_ = rad2deg(j2_rad);
    C_deg_ = rad2deg(j3_rad) + 180.0;
    alg_positive_operation(A_deg_, B_deg_, C_deg_, &x_mm_, &y_mm_, &z_mm_);

    // 发布末端位置（mm → m）
    geometry_msgs::msg::PointStamped ee_msg;
    ee_msg.header.stamp = this->now();
    ee_msg.header.frame_id = "Fixed_Base";  // 或 base_link
    ee_msg.point.x = x_mm_ / 1000.0;
    ee_msg.point.y = y_mm_ / 1000.0;
    ee_msg.point.z = z_mm_ / 1000.0;
    ee_pub_->publish(ee_msg);

    // 4) 发布 JointTrajectory 作为当前位置指令
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0);  // 立即执行
    traj.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {j1_rad, j2_rad, j3_rad};
    // 单点轨迹必须给非零 duration，否则 JointTrajectoryController 会判定
    // "time_from_start 必须大于 0" 并拒绝整条轨迹
    pt.time_from_start = rclcpp::Duration::from_seconds(dt_);

    traj.points.push_back(pt);
    traj_pub_->publish(traj);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "EE[mm]=(%.1f, %.1f, %.1f), A,B,C[deg]=(%.2f, %.2f, %.2f), "
                         "J(rad)=[%.3f, %.3f, %.3f]",
                         x_mm_, y_mm_, z_mm_,
                         A_deg_, B_deg_, C_deg_,
                         j1_rad, j2_rad, j3_rad);
  }

  // ===== ROS 成员 =====
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ee_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;

  // 当前“理论末端位置”（mm）
  double x_mm_{0.0}, y_mm_{0.0}, z_mm_{0.0};

  // 当前几何角（deg）
  double A_deg_{90.0}, B_deg_{0.0}, C_deg_{180.0};

  // 末端速度（mm/s）
  double vx_mm_s_{0.0}, vy_mm_s_{0.0}, vz_mm_s_{0.0};

  // 控制周期（s）
  double dt_{0.02};

  // J1 的限位（rad）
  double j1_min_rad_{-deg2rad(35.0)};
  double j1_max_rad_{ deg2rad(35.0)};

  // ===== 新增：IK 日志相关 =====
  std::string log_path_{"ik_log.csv"};
  std::ofstream ik_log_;
  bool log_enabled_{false};
  rclcpp::Time t0_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToVelocity>());
  rclcpp::shutdown();
  return 0;
}
