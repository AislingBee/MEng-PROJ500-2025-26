# Software
This folder contains all the ROS2 software for communicating with the various comoputers and external sensors.

## Folder Contents:

<img width="759" height="343" alt="ros2 motor test 1 drawio" src="https://github.com/user-attachments/assets/6acf4cdb-a618-4b3c-b4a8-d8a31d1f741e" />

## Launch Files:
### Note, none of this code has yet to be tested on a physical motor and all values are hard coded or randomly generated at this point

`motor_control_launch.py` <- this will launch the main motor control scripts including getting the input from all 12 motors and returning CAN messages back

Params that may be changed as needed:

`motor_test_launch.py` <- this will launch the  motor control test script this should only run the code to recieve info from 1 motor and return commands to that singular one

Params that may be changed as needed:

## How to Run ROS2:

```
colcon build --packages-select motor_test
source install/setup.bash
ros2 run <launch file name.py>
```
