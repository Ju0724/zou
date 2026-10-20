from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 注意：robot_description / robot_state_publisher 由
    # dolphin_arm_description/launch/gazebo.launch.py 提供，这里不要重复启动，
    # 否则会出现两个同名 robot_state_publisher 抢着发布 TF。
    # 另外 controller_manager 由 gazebo_ros2_control 插件创建，必须先把 Gazebo
    # 和模型拉起来再运行本文件（bringup 里用 6s 延时保证顺序）。

    # joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # arm_controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # gripper_controller
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
