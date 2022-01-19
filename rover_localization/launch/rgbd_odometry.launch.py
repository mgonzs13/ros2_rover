from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{
        "frame_id": "base_link",
        "subscribe_depth": True,
        "approx_sync": False,
        "publish_tf": False,
        "wait_imu_to_init": False}]

    remappings = [
        ("rgb/image", "/camera/image_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("imu", "/imu")]

    return LaunchDescription([

        Node(
            package="rtabmap_ros",
            executable="rgbd_odometry",
            namespace="rtab",
            output="log",
            parameters=parameters,
            remappings=remappings,
            arguments=['--ros-args', '--log-level', "Warn"]),

        # Node(
        #     package="rtabmap_ros",
        #     executable="rtabmapviz",
        #     output="screen",
        #     parameters=parameters,
        #     remappings=remappings),
    ])
