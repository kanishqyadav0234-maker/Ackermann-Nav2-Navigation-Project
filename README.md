# Ackermann Robot Autonomous Navigation using SLAM & Nav2 (ROS2)

This project demonstrates complete autonomous navigation of an **Ackermann steering robot** in a custom Gazebo maze environment using:

- SLAM Toolbox (Mapping)
- AMCL (Localization)
- Nav2 Stack (Path Planning & Navigation)

The robot creates its own map and then navigates autonomously to a goal inside the environment.

---

#  Project Features

✔ Custom Gazebo Maze World  
✔ SLAM Mapping  
✔ Map Saving  
✔ AMCL Localization  
✔ Nav2 Path Planning  
✔ Autonomous Navigation  

---
# Nav2 Stack

## Step 1 — Launch Gazebo

```bash
cd ~/arkermann_ws
source install/setup.bash
ros2 launch arkermann_bringup gazebo.launch.py
```
## Step 2 — Run Localization (AMCL)
```bash
cd ~/arkermann_ws
source install/setup.bash
ros2 launch nav2_bringup localization_launch.py map:=/home/kanishq/arkermann_ws/maps/arkermann.yaml use_sim_time:=true 

```
## Step 3 — Rviz2
```bash
cd ~/arkermann_ws
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```
## Step 4 - Run Navigation
```bash
cd ~/arkermann_ws
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
## Navigation Steps

- Click 2D Pose Estimate
- Set Initial Pose
- Click Nav2 Goal
- Set Goal Location
- Robot will autonomously navigate.
---










