# tracking_navigation
This is a ROS package which is meant to regulate robot speed based on its distance from a certain object. Currently this ROS package only calculates relative distance between the robot and the object in a Gazebo environment.

# REQUIREMENTS
To use this package, you will need to have a gazebo world containing a URDF robot running. A good example of such world can be obtained on the following link: https://github.com/romb-technologies/ariac_environment/tree/master/models/ariac

# SETUP
To use this package, create a catkin workspace and place this repository into *src* folder in your catkin workspace. Then simply run `catkin_make` in workspace root folder.

# USAGE
Package consists of three launch files.
The first one, `fake_tracking.launch` creates a tf node in a tf tree from a certain gazebo and then extracts a transform between that node and a robot (base_link) node.
To run the launch file use the following command:
>roslaunch tracking_navigation fake_tracking.launch object_name:="`object_name`" global_frame:="`global_frame`" tracking_frame:="`tracking_frame`"

where:
  - `object_name` denotes a name of an object inside a Gazebo world which we want to connect to a tf tree, default value = `animated_box` (will be changed in future versions)
  - `global_frame` denotes a name of root tf tree node, or a tf tree node in general, which will be a parent to a Gazebo object node in tf tree, default value = `pioneer/map`
  - `tracking_frame` denotes a robot node in tf tree, which will be used for transform calculation, default value = `pioneer/base_link`

`move_base_pioneer.launch` uses `ROS move_base` package (http://wiki.ros.org/move_base) to allow a robot to reach a given goal in the world. To configure the package to your needs, configuration files are located in *move_base_params* folder. There is a twist, however, in a way that the velocity the move_base package publishes to the robot is being intercepted and modified by a python script `velocity_refactoring.py`. The script, depending on the transform received from the previous launch file through `relative_pose` topic, modifies robot's velocity by a certain factor.
To run this launch file, use the following command:
>roslaunch tracking_navigation move_base_pioneer

The third launch file, `tracking_navigation.launch` simply runs both of the previously mentioned launch files, with a command similar to the first one:
>roslaunch tracking_navigation tracking_navigation.launch object_name:="`object_name`" global_frame:="`global_frame`" tracking_frame:="`tracking_frame`"

Explanations for the parameters are the same.
