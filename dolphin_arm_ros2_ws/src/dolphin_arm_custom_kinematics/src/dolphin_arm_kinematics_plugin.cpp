#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_list_macros.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace dolphin_arm_kinematics
{

// ---------- 基础工具 ----------
static inline double deg(double r){ return r * 180.0 / M_PI; }
static inline double rad(double d){ return d * M_PI / 180.0; }

// ==============================================================
// 几何参数（单位：mm / deg）
// 与 dolphin_arm_joy/src/joy_to_velocity.cpp 中的模型保持一致
// ==============================================================
static constexpr double kBaseOffset   = 67.0;         // 7 + 60
static constexpr double kShoulderY    = 65.5;
static constexpr double kUpperArm     = 135.0;
static constexpr double kForearm      = 145.0;
static constexpr double kAMin         = 0.0;
static constexpr double kAMax         = 180.0;

// ---------- 正运动学：几何角 A,B,C（deg） -> 末端 xyz（mm） ----------
static void forwardKinematicsDeg(double a, double b, double c,
                                 double& x, double& y, double& z)
{
  const double theta = rad(a - 35.0);            // A 的传动映射
  const double inner = kUpperArm * std::cos(rad(111.0 - b))
                     + kForearm  * std::sin(rad(c - b + 21.0))
                     + kBaseOffset;

  x = std::cos(theta) * inner;
  y = kShoulderY
    + kUpperArm * std::sin(rad(111.0 - b))
    - kForearm  * std::cos(rad(c - b + 21.0));
  z = std::sin(theta) * inner;
}

// ---------- 联动约束：C 随 B 变化 ----------
static inline bool checkJointCouplingDeg(double b, double c)
{
  const double minC = 140.0 - b;
  const double maxC = std::min(180.0, 196.0 - b);
  return (c >= minC) && (c <= maxC);
}

// ---------- 逆运动学：末端 xyz（mm） -> 几何角 A,B,C（deg） ----------
// 返回距离 seed（几何角）最近的一组解；无解返回 false
static bool inverseKinematicsMm(double x, double y, double z,
                                double seedA, double seedB, double seedC,
                                double& outA, double& outB, double& outC)
{
  const double R = std::sqrt(x * x + z * z);
  if (R < 1e-6)
    return false;

  bool   found = false;
  double bestCost = std::numeric_limits<double>::max();

  // inner 符号对应 "arm 朝前 / 朝后" 两支
  for (int sign = 1; sign >= -1; sign -= 2)
  {
    const double inner = sign * R;
    const double theta = std::atan2(z / inner, x / inner);  // rad

    const double U = inner - kBaseOffset;
    const double V = y - kShoulderY;
    const double L = std::hypot(U, V);

    if (L > kUpperArm + kForearm + 1e-6)  // 超出工作空间
      continue;
    if (L < std::fabs(kForearm - kUpperArm) - 1e-6)
      continue;

    const double phi = std::atan2(V, U);
    const double cosD = std::clamp(
        (kUpperArm * kUpperArm + L * L - kForearm * kForearm) / (2.0 * kUpperArm * L),
        -1.0, 1.0);
    const double d2 = std::acos(cosD);

    for (int elbow = 1; elbow >= -1; elbow -= 2)
    {
      const double p = phi + elbow * d2;
      const double elbowX = U - kUpperArm * std::cos(p);
      const double elbowY = V - kUpperArm * std::sin(p);
      const double r = std::atan2(elbowY, elbowX);

      double a = std::fmod(deg(theta) + 35.0 + 360.0, 360.0);
      const double b = 111.0 - deg(p);
      const double c = deg(r) + b + 69.0;

      if (a < kAMin || a > kAMax)
        continue;
      if (!checkJointCouplingDeg(b, c))
        continue;

      const double cost = std::pow(a - seedA, 2)
                        + std::pow(b - seedB, 2)
                        + std::pow(c - seedC, 2);
      if (cost < bestCost)
      {
        bestCost = cost;
        outA = a; outB = b; outC = c;
        found = true;
      }
    }
  }
  return found;
}

// ==============================================================
class DolphinArmIK : public kinematics::KinematicsBase
{
public:
    // MoveIt2 (Humble 及以后) 的初始化接口：
    // initialize(node, robot_model, group_name, base_frame, tip_frames, search_discretization)
    bool initialize(const rclcpp::Node::SharedPtr& node,
                    const moveit::core::RobotModel& robot_model,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::vector<std::string>& tip_frames,
                    double /*search_discretization*/) override
    {
      if (tip_frames.empty())
      {
        RCLCPP_ERROR(node->get_logger(), "DolphinArmIK: 没有收到 tip frame");
        return false;
      }

      const moveit::core::JointModelGroup* jmg = robot_model.getJointModelGroup(group_name);
      if (jmg == nullptr)
      {
        RCLCPP_ERROR(node->get_logger(), "DolphinArmIK: 找不到规划组 '%s'", group_name.c_str());
        return false;
      }
      if (jmg->getVariableCount() != 3)
      {
        RCLCPP_ERROR(node->get_logger(),
                     "DolphinArmIK: 规划组 '%s' 有 %zu 个自由度，本插件只支持 3 自由度",
                     group_name.c_str(), jmg->getVariableCount());
        return false;
      }

      node_ = node;
      group_name_ = group_name;
      base_frame_ = base_frame;
      tip_link_ = tip_frames.front();
      // 关节名从 RobotModel 取，避免和 URDF 脱节
      joint_names_ = jmg->getVariableNames();
      link_names_ = { tip_link_ };
      initialized_ = true;

      RCLCPP_INFO(node_->get_logger(),
                  "DolphinArmIK initialized: group='%s', base='%s', tip='%s', joints=[%s, %s, %s]",
                  group_name.c_str(), base_frame.c_str(), tip_link_.c_str(),
                  joint_names_[0].c_str(), joint_names_[1].c_str(), joint_names_[2].c_str());
      return true;
    }


  // ---------- IK ----------
  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                     const std::vector<double>& seed_state,
                     std::vector<double>& solution,
                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& /*options*/) const override
  {
    if (!initialized_ || seed_state.size() < 3)
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return false;
    }

    // URDF 使用米，几何模型使用毫米
    const double x = ik_pose.position.x * 1000.0;
    const double y = ik_pose.position.y * 1000.0;
    const double z = ik_pose.position.z * 1000.0;

    // 关节角 -> 几何角，用于在多个解中挑选离种子最近的一个
    const double seedA = deg(seed_state[0]) + 90.0;
    const double seedB = deg(seed_state[1]);
    const double seedC = deg(seed_state[2]) + 180.0;

    double Adeg, Bdeg, Cdeg;
    if (!inverseKinematicsMm(x, y, z, seedA, seedB, seedC, Adeg, Bdeg, Cdeg))
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      return false;
    }

    // 几何角 -> 关节角（与 URDF / joy_to_velocity 的映射一致）
    solution = { rad(Adeg - 90.0), rad(Bdeg), rad(Cdeg - 180.0) };
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  // ---------- FK ----------
  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override
  {
    poses.clear();
    if (link_names.size() != 1 || link_names[0] != tip_link_)
      return false;
    if (joint_angles.size() < 3) return false;

    // 关节角 -> 几何角
    const double a = deg(joint_angles[0]) + 90.0;
    const double b = deg(joint_angles[1]);
    const double c = deg(joint_angles[2]) + 180.0;

    double x, y, z;
    forwardKinematicsDeg(a, b, c, x, y, z);

    geometry_msgs::msg::Pose p;
    p.position.x = x / 1000.0;
    p.position.y = y / 1000.0;
    p.position.z = z / 1000.0;
    // 本插件只做位置 IK，末端姿态不参与求解；这里返回单位四元数，
    // 因此该插件不能用于带姿态约束的目标，详见 README。
    p.orientation.w = 1.0;
    p.orientation.x = p.orientation.y = p.orientation.z = 0.0;
    poses.push_back(p);
    return true;
  }

  // ---------- MoveIt2 需要的 searchPositionIK 接口 ----------
  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    (void)ik_seed_state; (void)timeout;
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    (void)consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    bool success = searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
    if (success && solution_callback)
      solution_callback(ik_pose, solution, error_code);
    return success;
  }

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout,
                        const std::vector<double>& consistency_limits,
                        std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    bool success = searchPositionIK(ik_pose, ik_seed_state, timeout,
                                    consistency_limits, solution, error_code, options);
    if (success && solution_callback)
      solution_callback(ik_pose, solution, error_code);
    return success;
  }

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }

  const std::vector<std::string>& getLinkNames() const override
  {
    if (link_names_.empty())
      link_names_.push_back(tip_link_);
    return link_names_;
  }

private:
  mutable std::vector<std::string> joint_names_;
  mutable std::vector<std::string> link_names_;
  rclcpp::Node::SharedPtr node_;
  std::string group_name_, base_frame_, tip_link_;
  bool initialized_ = false;
};

} // namespace dolphin_arm_kinematics

PLUGINLIB_EXPORT_CLASS(dolphin_arm_kinematics::DolphinArmIK, kinematics::KinematicsBase)
