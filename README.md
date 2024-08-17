# pathplanning_go1

# System Setup
1. Ubuntu 18.04
2. Install [ROS](http://wiki.ros.org/) Melodic.
3. Create your own catkin_ws and clone the repositories.
4. Unitree Go1 install: https://github.com/unitreerobotics/unitree_ros
5. open manipulator install: https://github.com/ROBOTIS-GIT/open_manipulator
6. Slamtec LiDAR install: https://www.slamtec.com/cn/support#rplidar-mapper
7. [pathplanning_go1](https://github.com/vchint6/pathplanning_go1) contains modified files
8. Build (catkin_make)


# Simulation
1. launch the simulation world - roslaunch unitree_navigation my_gazebo.launch
2. run the go1 controller - ./devel/lib/unitree_guide/junior_ctrl
3. launch the SLAM for mapping - roslaunch unitree_navigation my_slam.launch
4. save map - rosrun map_server map_saver -f ~/catkin_ws/src/unitree_guide/unitree_navigation/maps/sim_map
5. launch navigation file - roslaunch unitree_navigation my_navigation.launch


# Ball tracking Simulation
1. run commands upto navigation in separate terminals
2. rosrun opencv detect_ball.py --ros-args -p image_in:=/camera_face/color/image_raw
3. rosrun opencv detect_ball_3d.py
4. rosrun opencv follow_ball.py


# Manipulator Controller
1. launch manipulator controller - roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
2. run manipulator gui contoller - roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
   

