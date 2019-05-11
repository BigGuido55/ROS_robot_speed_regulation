# ROS_robot_speed_regulation
This is a ROS package which is meant to regulate robot speed based on its distance from a certain object. Currently this ROS package only calculates relative distance between the robot and the object.

To use this package, create a catkin workspace and place the folder "robot_movement_control" into "src" folder and run "catkin_make in workspace's root folder.

Package can then be launched with:
roslaunch robot_movement_control initalLaunch.launch

