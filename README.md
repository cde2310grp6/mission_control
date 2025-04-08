# CDE2310 mission_control (MAIN) node

This node handles the Finite State Machine (FSM) flow of events during our mission, integrating the nodes packaged in the other repos, such as 
 - casualty_location
 - nav2_wfd


## Commands to Run

### Run on Remote PC
1. (if running on simulation) Turtlebot3 Gazebo launch
  ```bash
  ros2 launch mission_control turtlebot3_world.launch.py
  ```

2. nav2 Stack
 ```bash
  ros2 launch mission_control navigation.launch.py use_sim_time:=True
  ```
Parameters for the Nav2 stack can be found in mission_control/config/nav2_params.yaml

3. slam_toolbox online_async
 ```bash
 ros2 launch mission_control online_async.launch.py
 ```

4. (if you need to debug) RViz
 ```bash
 ros2 launch mission_control rviz.launch.py 
 ```

5. Finally, run the missionStart launch file
```bash
ros2 launch mission_control mission_start.launch.py
```

### Run on SBC
1. turtlebot bringup (initialises lidar, etc.)
```bash
ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=False
```

2. amg8833 driver and heat source yaw publisher
```bash
ros2 run casualty_location ir_pub
```

3. turtlebot_launcher service
```bash
ros2 run turtlebot_launcher launcher_service
```


## Dependencies

### Clone and Build on Remote PC
clone the following directories into the /ros2_ws/src folder
```bash
git clone https://github.com/cde2310grp6/mission_control.git
```
```bash
git clone https://github.com/cde2310grp6/custom_msg_srv.git
```
```bash
git clone https://github.com/cde2310grp6/casualty_location.git
```
```bash
git clone https://github.com/cde2310grp6/nav2_wavefront_frontier_exploration.git
```

### Clone and Build on SBC
```bash
git clone https://github.com/cde2310grp6/casualty_location.git
```
```bash
https://github.com/cde2310grp6/turtlebot_launcher.git
```

ROS2 Humble
Nav2 Stack
for Gazebo, Gazebo Classic 11.14.0 tested and working


We used the ROBOTIS turtlebot3 manual for majority of the necessary setup.
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
