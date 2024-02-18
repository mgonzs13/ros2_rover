# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_path = get_package_share_directory("rover_gazebo")
    pkg_gazebo_ros = get_package_share_directory("ros_gz_sim")
    pkg_rover_localization = get_package_share_directory("rover_localization")
    pkg_rover_navigation = get_package_share_directory("rover_navigation")

    rviz_config = os.path.join(
        pkg_path,
        "rviz",
        "default.rviz")

    ### ARGS ###
    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_path, "worlds", "empty.world"),
        description="Gazebo world")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether launch rviz2")

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
        default_value="0.22",
        description="Initial pose z")

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw")

    nav2_planner = LaunchConfiguration("nav2_planner")
    nav2_planner_cmd = DeclareLaunchArgument(
        "nav2_planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)")

    nav2_controller = LaunchConfiguration("nav2_controller")
    nav2_controller_cmd = DeclareLaunchArgument(
        "nav2_controller",
        default_value="RPP",
        choices=["RPP", "TEB"],
        description="Nav2 controller (RPP or TEB)")

    ### NODES ###
    rviz_cmd = Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config,
                   "--ros-args", "--log-level", "Error"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([launch_rviz])),
    )

    gz_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist"
        ],
        parameters=[os.path.join(get_package_share_directory(
            "rover_gazebo"), "config", "gz_bridge.yaml")],
        remappings=[
            ("/camera/image", "/camera/rgb/image_raw"),
            ("/camera/depth_image", "/camera/depth/image_raw"),
            ("/camera/camera_info", "/camera/camera_info"),
            ("/camera/points", "/camera/points")
        ],
        output="screen"
    )

    ### LAUNCHS ###
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, "launch", "gz_sim.launch.py")
        ]),
        launch_arguments=[("gz_args", [
            world,
            " -v 4",
            " -r",
            " --gui-config ", os.path.join(pkg_path, "gui", "gui.config")
        ])]
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch",
                         "localization.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_navigation, "launch",
                         "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "planner": nav2_planner,
            "controller": nav2_controller
        }.items()
    )

    cmd_vel_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, "launch/include",
                         "cmd_vel.launch.py")
        ),
    )

    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, "launch/include", "spawn.launch.py")
        ),
        launch_arguments={
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_z": initial_pose_z,
            "initial_pose_yaw": initial_pose_yaw
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(launch_rviz_cmd)
    ld.add_action(world_cmd)
    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)
    ld.add_action(nav2_planner_cmd)
    ld.add_action(nav2_controller_cmd)

    ld.add_action(gazebo_cmd)
    ld.add_action(gz_bridge_cmd)
    ld.add_action(spawn_cmd)
    # ld.add_action(localization_cmd)
    # ld.add_action(navigation_cmd)
    # ld.add_action(cmd_vel_cmd)
    # ld.add_action(rviz_cmd)

    return ld
