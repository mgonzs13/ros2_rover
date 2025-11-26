# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

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
        description="Use simulation (Gazebo) clock if True",
    )

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Wheather to launch rtabmapviz",
    )

    parameters = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": False,
            "approx_sync": True,
            "publish_tf": True,
            "use_sim_time": use_sim_time,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_imu": 2,
            "qos_odom": 1,
            # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            "RGBD/Enabled": "true",
            "RGBD/OptimizeMaxError": "0.5",
            "RGBD/OptimizeFromGraphEnd": "false",
            "RGBD/CreateOccupancyGrid": "true",
            "RGBD/LoopClosureIdentityGuess": "false",
            "RGBD/LocalBundleOnLoopClosure": "false",
            "VhEp/Enabled": "false",
            "Rtabmap/CreateIntermediateNodes": "false",
            "GFTT/MinDistance": "7.0",
            "GFTT/QualityLevel": "0.001",
            "GFTT/BlockSize": "3",
            "GFTT/UseHarrisDetector": "true",
            "GFTT/K": "0.04",
            "BRIEF/Bytes": "64",
            # Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
            "Vis/EstimationType": "1",
            "Vis/ForwardEstOnly": "true",
            # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector
            "Vis/FeatureType": "8",
            "Vis/DepthAsMask": "true",
            "Vis/CorGuessWinSize": "40",
            "Vis/MaxFeatures": "0",
            "Vis/MinDepth": "0.0",
            "Vis/MaxDepth": "0.0",
            # 0=Features Matching, 1=Optical Flow
            "Vis/CorType": "0",
            # kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
            "Vis/CorNNType": "1",
            "Grid/Sensor": "1",
            "Grid/DepthDecimation": "4",
            "Grid/RangeMin": "0.0",
            "Grid/RangeMax": "5.0",
            "Grid/MinClusterSize": "10",
            "Grid/MaxGroundAngle": "45",
            "Grid/NormalK": "20",
            "Grid/ClusterRadius": "0.2",
            "Grid/CellSize": "0.1",
            "Grid/FlatObstacleDetected": "false",
            "Grid/RayTracing": "true",
            "Grid/3D": "true",
            "Grid/MapFrameProjection": "true",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "100.0",
            "GridGlobal/Eroded": "true",
            "GridGlobal/FloodFillDepth": "16",
        }
    ]

    remappings = [
        ("rgb/image", "camera/image_raw"),
        ("rgb/camera_info", "camera/camera_info"),
        ("depth/image", "camera/depth/image_raw"),
        ("imu", "imu"),
        ("odom", "odom"),
        ("goal", "goal_pose"),
    ]

    return LaunchDescription(
        [
            use_sim_time_cmd,
            launch_rtabmapviz_cmd,
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="log",
                parameters=parameters,
                remappings=remappings,
                arguments=["-d", "--ros-args", "--log-level", "Error"],
            ),
            Node(
                condition=IfCondition(launch_rtabmapviz),
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
        ]
    )
