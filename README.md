# RL_Homework4

## :hammer: Build

Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
 git clone https://github.com/MatteoRusso5/RL_HW4.git

```
Build your new package

```
 colcon build
```
Source the setup files

```
 source install/setup.bash
```
## :white_check_mark: Usage 🤖
If in any case the robot doesn't start moving, try to rerun
### 1. Autonomous navigation task with 4 different goals

Run Gazebo (REMEMBER TO CLICK PLAY)
```
 ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py

```
In the second terminal run the exploration
```
 ros2 launch rl_fra2mo_description fra2mo_explore.launch.py 
```
In the third terminal run the script for the goals
```
 ros2 run rl_fra2mo_description follow_waypoints.py 
```
You can also see the robot with all the frames on RVIZ (select tf)
```
 ros2 launch rl_fra2mo_description display_fra2mo.launch.py 
```
### 2. Autonomous navigation to get a complete map of the environment

Run Gazebo (REMEMBER TO CLICK PLAY)
```
 ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
In the second terminal run the exploration
```
 ros2 launch rl_fra2mo_description fra2mo_explore.launch.py 
```
In the third terminal run the script for the goals specifying the target
```
 ros2 run rl_fra2mo_description follow_waypoints.py --ros-args -p target:="map"
```
You can visualize the map on RVIZ launching
```
 ros2 launch rl_fra2mo_description display_fra2mo.launch.py map:=true
```
When the robot has reached the final goal, you can save the map in the desired folder (FIRST CHANGE DIRECTORY) using
```
ros2 run nav2_map_server map_saver_cli -f map
```

### 3. Vision-based navigation with aruco detection
Run Gazebo (REMEMBER TO CLICK PLAY)
```
 ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```
You can also see the robot with all the frames on RVIZ (select tf) (LAUNCH THIS BEFORE THE TOPIC ECHO)
```
 ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```
Launch the navigation and detection
```
ros2 launch rl_fra2mo_description vision_navigation.launch.py 
```
To visualize the Aruco pose published as TF
```
 ros2 topic echo /tf_static
```
To visualize the detection done by the camera (SELECT /aruco_single/result )
```
ros2 run rqt_image_view rqt_image_view 
```

