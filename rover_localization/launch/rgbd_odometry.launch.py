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
        "approx_sync_max_interval": 0.01,
        "publish_tf": False,
        "wait_imu_to_init": False,
        "publish_null_when_lost": False,
        "qos": 2,
        "qos_camera_info": 2,

        # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
        "Optimizer/Strategy": "2",
        "Optimizer/GravitySigma": "0.3",

        # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D
        "Odom/Strategy": "0",
        "Odom/ResetCountdown": "1",
        "Odom/Holonomic": "false",
        # 0=No filtering 1=Kalman filtering 2=Particle filtering
        "Odom/FilteringStrategy": "1",
        "Odom/ParticleSize": "500",
        "Odom/GuessMotion": "true",
        "Odom/AlignWithGround": "false",

        "GFTT/MinDistance": "2",
        "GFTT/QualityLevel": "0.1",
        "GFTT/BlockSize": "4",
        "GFTT/UseHarrisDetector": "false",
        "GFTT/K": "0.04",

        "SURF/Extended": "true",
        "SURF/HessianThreshold": "500",
        "SURF/Octaves": "4",
        "SURF/OctaveLayers": "4",
        "SURF/Upright": "false",
        "SURF/GpuVersion": "false",
        "SURF/GpuKeypointsRatio": "0.01",

        "SIFT/NFeatures": "1000",
        "SIFT/NOctaveLayers": "4",
        "SIFT/RootSIFT": "false",

        "FREAK/OrientationNormalized": "true",
        "FREAK/ScaleNormalized": "true",
        "FREAK/PatternScale": "30",
        "FREAK/NOctaves": "4",

        "KAZE/Extended": "true",
        "KAZE/Upright": "false",
        "KAZE/NOctaves": "4",
        "KAZE/NOctaveLayers": "4",
        # 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER
        "KAZE/Diffusivity": "1",

        "BRIEF/Bytes": "128",

        # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector
        "Vis/FeatureType": "8",
        "Vis/DepthAsMask": "true",
        "Vis/FeatureWindowSize": "10",
        "Vis/CorGuessWinSize": "40",
        "Vis/MaxFeatures": "1000",
        # 0=Features Matching, 1=Optical Flow
        "Vis/CorType": "0",
        # kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
        "Vis/CorNNType": "1"
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
