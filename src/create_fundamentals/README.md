# iRobot Create ROS 2 Driver

A ROS 2 driver for the iRobot Create mobile robot, including support for the Hokuyo URG laser scanner.

## 🚀 Running
### Laser Only
>ros2 launch create_fundamentals laser.launch.py
### Robot Only
>ros2 run create_fundamentals driver.py
### Robot + Laser
>ros2 launch create_fundamentals robot_and_laser.launch.py

### C++ Examples
>ros2 run create_fundamentals example_laser

>ros2 run create_fundamentals example_moving

>ros2 run create_fundamentals example_song

## ⚠️ Common Failure Cases

Error:

>[urg_node]: Error connecting to Hokuyo: Could not open serial Hokuyo

Cause: Wrong serial port.

Solution: Check your device path and update the launch argument if necessary. For example, change from the default /dev/ttyACM0 to /dev/ttyACM1:

>ros2 launch create_fundamentals laser.launch.py serial_port:=/dev/ttyACM1
