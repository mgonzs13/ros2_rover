# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{
        "frame_id": "base_link",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "approx_sync": True,
        "approx_sync_max_interval": 0.05,
        "publish_tf": False,
        "wait_imu_to_init": False,
        "publish_null_when_lost": False,
        "qos": 2,
        "qos_camera_info": 2,

        "Optimizer/Strategy": "2",
        "Optimizer/GravitySigma": "0.0",

        "Odom/Strategy": "0",
        "Odom/ResetCountdown": "1",
        "Odom/Holonomic": "false",
        "Odom/FilteringStrategy": "0",
        "Odom/ParticleSize": "500",

        "GFTT/MinDistance": "10",
        "GFTT/QualityLevel": "0.1",

        "SURF/Extended": "true",
        "SURF/Upright": "true",

        "SIFT/NFeatures": "500",
        "SIFT/NOctaveLayers": "5",

        "FREAK/OrientationNormalized": "true",
        "FREAK/ScaleNormalized": "true",

        "KAZE/Extended": "true",
        "KAZE/Upright": "true",

        "Vis/CorGuessWinSize": "40",
        "Vis/CorType": "0",
        "Vis/MaxFeatures": "1000",
        "Vis/MinDepth": "0.0",
        "Vis/MaxDepth": "2.5",
    }]

    remappings = [
        ("rgb/image", "camera/image_raw"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "imu"),
        ("odom", "odom_rgbd")
    ]

    return LaunchDescription([

        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            output="log",
            parameters=parameters,
            remappings=remappings,
            arguments=["--ros-args", "--log-level", "Warn"]),
    ])
