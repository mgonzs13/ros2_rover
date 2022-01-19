import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
import xacro


def generate_launch_description():

    xacro_file = os.path.join(get_package_share_directory(
        "rover_description"), "robots/rover.urdf.xacro")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {"robot_description": doc.toxml(), "use_sim_time": True}

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="0.0",
        description="Initial pose x")

    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="0.0",
        description="Initial pose y")

    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z",
        default_value="0.0",
        description="Initial pose z")

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw")

    robot_state_publisher_cmd = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "rover",
                   "-topic", "robot_description",
                   "-timeout", "120",
                   "-x", initial_pose_x,
                   "-y", initial_pose_y,
                   "-z", initial_pose_z,
                   "-Y", initial_pose_yaw],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    ld = LaunchDescription()

    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(joint_trajectory_controller_spawner)
    ld.add_action(velocity_controller_spawner)

    return ld
