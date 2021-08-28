# ros2_rover

This is a ROS 2 version of the [Probable Adventure Project](https://github.com/gadiego92/probable-adventure). A [C++](./rover_motor_controller_cpp) version and a [Python](./rover_motor_controller) version are included. Finally, there is a PS3 joy controller and a hokuyo laser can be used.


## Dependencies
```shell
sudo apt install ros-foxy-joy-linux ros-foxy-teleop-twist-joy ros-foxy-urg-node
```

## Usage
Clone the repository:
```shell
cd ros2_ws
git clone https://niebla.unileon.es/mgonzs/ros2_rover
```

Compile using colcon:
```shell
colcon build
source install/setup.bash
```

Launch the project:
```shell
ros2 launch rover rover_launch.py
```

## Rover Service
A Linux service can be created to control the execution of this project and execute it at boot time. To create it, the following commands are used:
```shell
cd rover_service
sudo ./install.sh
```
