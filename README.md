# DDMR
Differential Drive Mobile Robot Simulation. Implements PRM navigation in ROS2.

## Launch 
```
ros2 launch project_4 launch.py robot:=(robot_file_path | default=robots/normal.robot)
```

## RVIZ
1. Launch rviz
```
ros2 run rviz2 rviz2
```
2. Add Robot Model and subscribe to /robot_description
3. Change Global Options Fixed Frame and Grid Reference Frame to world
4. Add TF with base_link, heading_box, laser, and world frames enabled 
