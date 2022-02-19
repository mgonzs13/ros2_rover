from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Wheather to launch rtabmapviz")

    parameters = [{
        "frame_id": "base_link",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan": False,
        "approx_sync": True,
        "publish_tf": True,
        "use_sim_time": use_sim_time,

        "Optimizer/GravitySigma": "0",
        "Optimizer/Strategy": "1",
        "RGBD/OptimizeMaxError": "1.0",
        "Vis/MaxFeatures": "500",

        "GFTT/MinDistance": "10",
        "GFTT/QualityLevel": "0.00001",
        "Vis/CorGuessWinSize": "40",

        "Grid/MinClusterSize": "20",
        "Grid/MaxGroundAngle": "35",
        "Grid/RangeMax": "5",
        "Grid/NormalK": "10",
        "Grid/CellSize": "0.1",
        "Grid/FlatObstacleDetected": "false",
        "GridGlobal/MinSize": "200"
    }]

    remappings = [
        ("rgb/image", "camera/image_raw"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "imu"),
        ("odom", "odom")]

    return LaunchDescription([
        use_sim_time_cmd,
        launch_rtabmapviz_cmd,

        Node(
            package="rtabmap_ros",
            executable="rtabmap",
            output="screen",
            parameters=parameters,
            remappings=remappings,
            arguments=["-d",
                       "--ros-args", "--log-level", "Warn"]),

        Node(
            condition=IfCondition(launch_rtabmapviz),
            package="rtabmap_ros",
            executable="rtabmapviz",
            output="screen",
            parameters=parameters,
            remappings=remappings),
    ])
