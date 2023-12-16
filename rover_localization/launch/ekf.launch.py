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


from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    params_file = os.path.join(get_package_share_directory(
        "rover_localization"), "config", "ekf.yaml")

    param_substitutions = {
        "use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="log",
        parameters=[configured_params],
        remappings=[("odometry/filtered", "/odom"),
                    ("accel/filtered", "/accel")])

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)

    ld.add_action(ekf_cmd)

    return ld
