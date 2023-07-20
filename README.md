# Loop-O Gripper Ros Integration

This ROS2 package contains the interfaces and driver required to control the gripper.

## Setup Instructions

1. Clone this repository in the src directory of your ROS2 workspace.

```$ cd my_ROS2_ws/src
$ git clone "loopo_driver_node = loopo_driver.loopo_driver_node:main"
$ git checkout main
```

1. Build and source your workspace.

```
$ cd ..
$ colcon build --symlink-install
$ source install/setup.sh
```

1. Connect the gripper to the computer through usb and launche one of the provided nodes.

```ros2 run loopo_gripper loopo_action_servers_node```

if you get a serial exception error it means the gripper is not using the default serial interface `dev/ttyACMO`</br>
You will need to find what interface the gripper is using and modify the following line:</br>
`383-    loopo = LoopODriver(com_port="/dev/ttyACM0")`</br>
in file `loopo_gripper/loopo_driver/loopo_driver/loopo_action_servers.py`
if you now run the node again it should start all the actions servers.

## Possible Interfaces

The package offers 2 ways to control the gripper:</br>

### Loopo_driver_node

Loopo_driver_node instantiates a service that lets the user send command directly to the gripper and responds with the status of the gripper.</br>

```
int8    id
int8    command
float32 value
---
int32   ex_position
int8    ex_status
int8    ex_control

int32   tw_size
int32   tw_offset
int8    tw_status
int8    tw_control

int32   lp_size
float32 lp_force
int8    lp_status
int8    lp_control
```

more information about the structure of the command and response can be found in [firmware documentation](https://github.com/itsameWolf/loopo-firmware).
The service is blocking utill the response is received from the gripper, this should take less than 20 ms.

### Loopo_action_server

the loop_action_servers node instantiates action servers for all actions for all the devices on the girpper. theses include moving the various devices as well as grasping and twisting.
