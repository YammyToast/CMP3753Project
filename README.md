# CMP3753 Project 
----

EXPORT EVERYTHING BEFORE STARTING!

----


# Backtracking Spiral Algorithm
https://wiki.ros.org/full_coverage_path_planner

# Turtlebot3 Gazebo World
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Turtlebot3 Cartographer Implementation
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Turtlebot3 Prebuilt Navigation (requires map.yaml from cartographer)
# rviz set goal to start navigation.
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=assets/my_map.yaml

# Save Cartographer map for navigation
ros2 run nav2_map_server map_saver_cli -f my_map

# Import Turtlebot models to gazebo
export GAZEBO_MODEL_PATH=`ros2 pkg \
prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/

# Set turtlebot model to use in gazebo (this also sets up topics)
export TURTLEBOT3_MODEL=burger

# ref
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html