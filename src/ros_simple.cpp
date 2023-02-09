#include "ros_simple/ros_simple.hpp"

namespace rossimple{
Node::Node(std::string node_name) :
	#ifdef ROS1
	nh("~")
	#endif
	#ifdef ROS2
	rclcpp::Node(node_name)
	#endif
	{
	
	// Nothing
}

Node::~Node()
{
	// Nothing
}
} // End Namespace rossimple
