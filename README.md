# tracking_navigation
This is a ROS package which is meant to regulate robot speed based on its distance from a certain object. Currently this ROS package only calculates relative distance between the robot and the object in a Gazebo environment.

# REQUIREMENTS
To use this package, you will need to have a gazebo world containing a URDF robot running. A good example of such world can be obtained on the following link: https://github.com/romb-technologies/ariac_environment/tree/master/models/ariac

# SETUP
To use this package, create a catkin workspace and place this repository into "src" folder. Then simply run "catkin_make" in workspace's root folder.

To launch this package, use the following command:
roslaunch ROS_robot_speed_regulation initialLaunch.launch object_name:="*object_name*" global_frame:="*global_frame*" tracking_frame:="*tracking_frame*"

where:

  -*object_name* denotes a name of an object inside a Gazebo world, this argument is used to extract transform of an object from one of Gazebo's rostopics and will be used to connect that object's transform to the root tf tree node
  
  -*global_frame* denotes a name of root tf tree node, robot must already be connected to this node
  
  -*tracking_frame* denotes a name of robots' tf tree node, this package will calculate distance between this node and object_name node

These arguments have their default values, being:
	*object_name* - "animated_box"	("will be changed in future versions")
	*global_frame* - "pioneer/map"
	*tracking_frame* - "pioneer/odom"
