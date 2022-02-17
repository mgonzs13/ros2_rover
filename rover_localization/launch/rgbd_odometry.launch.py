from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{
        "frame_id": "base_link",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "approx_sync": True,
        "publish_tf": False,
        "wait_imu_to_init": False}]

    remappings = [
        ("rgb/image", "camera/image_raw"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "imu"),
        ("odom", "odom_rgbd")]

    return LaunchDescription([

        Node(
            package="rtabmap_ros",
            executable="rgbd_odometry",
            output="log",
            parameters=parameters,
            remappings=remappings,
            arguments=["--ros-args", "--log-level", "Warn"]),
    ])
