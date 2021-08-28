# ros2_rover

This is a ROS 2 (Foxy) version of the [Probable Adventure Project](https://github.com/gadiego92/probable-adventure). A [C++](./rover_motor_controller_cpp) version and a [Python](./rover_motor_controller) version of the lx16a controller are included. Besides, a [PS3 joy controller](./rover/launch/joy_teleop_launch.py) and a [hokuyo laser](./rover/launch/urg_node_launch.py) can be used.

<div align="center">
    <img src="rover.png" width="50%"/>
</div>

## Dependencies
```shell
sudo apt install ros-foxy-joy-linux ros-foxy-teleop-twist-joy ros-foxy-urg-node
```

## Usage
Clone the repository:
```shell
cd ~/ros2_ws/src
git clone https://niebla.unileon.es/mgonzs/ros2_rover
```

Compile using colcon:
```shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Launch the project:
```shell
source /opt/ros/foxy/setup.bash
ros2 launch rover rover_launch.py
```

## Rover Service
A Linux service can be created to control the execution and launch everything at boot time. To create the rover service, the following commands are used:
```shell
cd rover_service
sudo ./install.sh
```

Check rover service:
```shell
sudo service rover status
```
