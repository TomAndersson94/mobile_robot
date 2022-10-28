# Mobile robot - readme under construction.

##
A homemade mobile robot to experiment with SLAM and path planning. The project is still in its early stages.

## Important notes:

- Raspberry ip: 192.168.0.180
- Laptop ip: 192.168.0.192
- SSH from laptop by command: ssh ubuntu@192.168.0.180
- Raspberry runs roscore
- Mobile uses the app ROS-Mobile to communicate
- Lidar has been tested, works fine. Had to config usb permissions. (Make sure to not use a power only usb cable)
- Run command source devel/setup.bash in catkin_ws where ros is running. Automate this!
- Run command export ROS_MASTER_URI=http://192.168.0.180:11311 on laptop to set raspberry as ros master (all terminals) Automate this!
- Run command export ROS_IP='ip of node'. eg export ROS_IP=192.168.0.180 on raspberry, in all terminals that uses ros. To make publishing and subscribing over different machines work. Automate this!
- To test lidar before fixed frame has been fixed run rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map laser 100
- To run lidar run command roslaunch rplidar_ros rplidar.launch

## Stuff thats implemented
0. Source and order components
1. Install Ubuntu Server 20.04 and ROS Noetic on Raspberry and config wifi and ssh.
2. Design and print body
3. Mount components
4. Connect components
5. Implement control of motors via joystick on mobile.
6. Implement reading of speed sensors.

## Next steps
0. Create propper launchfile and classes.
1. Write Bash-script for automated startup
2. Create urdf file for robot
3. Implement controller for wheel speed
4. SLAM
5. Implement path planning with A*
6. Implement MPC for following trajectory

