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
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_vault_localization = get_package_share_directory("vault_localization")
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

    launch_gui = LaunchConfiguration("launch_gui")
    launch_gui_cmd = DeclareLaunchArgument(
        "launch_gui",
        default_value="True",
        description="Whether launch gzclient")

    pause_gz = LaunchConfiguration("pause_gz")
    pause_gz_cmd = DeclareLaunchArgument(
        "pause_gz",
        default_value="False",
        description="Whether to pause gazebo")

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

    use_t265 = LaunchConfiguration("use_t265")
    use_t265_cmd = DeclareLaunchArgument(
        "use_t265",
        default_value="True",
        description="Wheter to use T265 camera or D435i")

    ### NODES ###
    rviz_cmd = Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([launch_rviz])),
    )

    ### LAUNCHS ###
    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(PythonExpression([launch_gui]))
    )

    gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "pause": pause_gz, }.items()
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vault_localization, "launch",
                         "localization.launch.py")
        ),
        launch_arguments={"use_sim_time": "True",
                          "namespace": "",
                          "use_visual_slam": "True",
                          "use_rgbd_odom": PythonExpression(["not ", use_t265])}.items()
    )

    fix_odom_frame_cmd = Node(
        package="vault_sensors",
        executable="fix_odom_frame_node",
        name="fix_odom_frame_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"frame_id": "base_link",
                     "odom_frame": "odom"}],
        remappings=[("odom_out", "odom"),
                    ("odom_in", "camera/pose/sample")],
        condition=IfCondition(PythonExpression([use_t265]))
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_navigation, "launch",
                         "bringup.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items()
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
            "initial_pose_yaw": initial_pose_yaw,
            "use_t265": use_t265
        }.items()
    )

    ld = LaunchDescription()

    ld.add_action(launch_gui_cmd)
    ld.add_action(pause_gz_cmd)
    ld.add_action(launch_rviz_cmd)
    ld.add_action(world_cmd)
    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)
    ld.add_action(use_t265_cmd)

    ld.add_action(gazebo_client_cmd)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(cmd_vel_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(fix_odom_frame_cmd)

    return ld
