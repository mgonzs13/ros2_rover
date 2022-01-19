
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    motors_command_parser_node_cmd = Node(
        name="motors_command_parser_node",
        package="rover_gazebo",
        executable="motors_command_parser_node",
        output="log",
    )

    vel_parser_node_cmd = Node(
        name="vel_parser_node",
        package="rover_motor_controller_cpp",
        executable="vel_parser_node",
        output="log",
    )

    ld = LaunchDescription()

    ld.add_action(motors_command_parser_node_cmd)
    ld.add_action(vel_parser_node_cmd)

    return ld
