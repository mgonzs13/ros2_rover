import os
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import xacro


def robot_state_publisher(context: LaunchContext, use_t265):

    ### XACRO ###
    use_t265_str = context.perform_substitution(use_t265)
    xacro_file = os.path.join(get_package_share_directory(
        "rover_description"), "robots/rover.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={"t265": use_t265_str})

    params = {"robot_description": doc.toxml(), "use_sim_time": True}

    ### NODES ###
    robot_state_publisher_cmd = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    return [robot_state_publisher_cmd]


def generate_launch_description():

    use_t265 = LaunchConfiguration("use_t265")
    use_t265_cmd = DeclareLaunchArgument(
        "use_t265",
        default_value="True",
        description="Wheter to use T265 camera or D435i")

    prepare_xacro_cmd = OpaqueFunction(
        function=robot_state_publisher, args=[use_t265])

    ld = LaunchDescription()
    ld.add_action(use_t265_cmd)
    ld.add_action(prepare_xacro_cmd)

    return ld
