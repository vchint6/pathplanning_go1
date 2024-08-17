# pathplanning_go1
1. Unitree Go1 quadruped connected to host PC via LAN
2. Slamtech Lidar connected to Head Nano 3
3. Open Manipulator-X connected to Head Nano 2
   ![image](https://github.com/user-attachments/assets/2aa1f43d-e5a2-4d2f-92c1-60d95b354887)
   ![image](https://github.com/user-attachments/assets/b4b7849f-63d9-46bf-9c75-2e389303ed14)
   ![image](https://github.com/user-attachments/assets/c54d074c-8ae6-4356-9d93-323d8a5af9f3)

# Host PC Setup
1. Ubuntu 18.04
2. Install [ROS](http://wiki.ros.org/) Melodic.
3. Create your own catkin_ws and clone the repositories.
4. Unitree Go1 install: https://github.com/unitreerobotics/unitree_ros
5. open manipulator install: https://github.com/ROBOTIS-GIT/open_manipulator
6. Slamtec LiDAR install: https://www.slamtec.com/cn/support#rplidar-mapper
7. [pathplanning_go1](https://github.com/vchint6/pathplanning_go1) contains modified files
8. Build (catkin_make)


# Navigation in Simulation World
1. launch the simulation world -
   ```bash
   roslaunch unitree_navigation my_gazebo.launch
3. run the go1 controller -
   ```bash
   ./devel/lib/unitree_guide/junior_ctrl
5. launch the SLAM for mapping -
   ```bash
   roslaunch unitree_navigation my_slam.launch
7. save map -
   ```bash
   rosrun map_server map_saver -f ~/catkin_ws/src/unitree_guide/unitree_navigation/maps/sim_map
9. launch navigation file -
   ```bash
   roslaunch unitree_navigation my_navigation.launch

# Ball tracking Simulation World
1. run commands upto navigation in separate terminals
   ```bash
   rosrun opencv detect_ball.py --ros-args -p image_in:=/camera_face/color/image_raw
   rosrun opencv detect_ball_3d.py
   rosrun opencv follow_ball.py


# Manipulator Controller
1. launch manipulator controller -
   ```bash
   roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
3. run manipulator gui contoller -
   ```bash
   roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
