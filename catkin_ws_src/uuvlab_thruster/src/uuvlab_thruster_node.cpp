
#include "ros/ros.h"
#include "std_msgs/String.h"

void DebugCallback0(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 0: [%s]", msg->data.c_str());
}

void DebugCallback1(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 1: [%s]", msg->data.c_str());
}

void DebugCallback2(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 2: [%s]", msg->data.c_str());
}

void DebugCallback3(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 3: [%s]", msg->data.c_str());
}

void DebugCallback4(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 4: [%s]", msg->data.c_str());
}

void DebugCallback5(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 5: [%s]", msg->data.c_str());
}

void DebugCallback6(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 6: [%s]", msg->data.c_str());
}

void DebugCallback7(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Debug Relay Thruster 7: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluerov2_thruster_node");
	ros::NodeHandle n;
	std::string robot_config;		
	ros::param::param<std::string>("~robot_config", robot_config,"heavy");
	ROS_INFO("Robot Configuration: %s",robot_config.c_str());
	ros::Subscriber sub0 = n.subscribe("thrusters/0/input",1000, DebugCallback0);
	ros::Subscriber sub1 = n.subscribe("thrusters/1/input",1000, DebugCallback1);
	ros::Subscriber sub2 = n.subscribe("thrusters/2/input",1000, DebugCallback2);
	ros::Subscriber sub3 = n.subscribe("thrusters/3/input",1000, DebugCallback3);
	ros::Subscriber sub4 = n.subscribe("thrusters/4/input",1000, DebugCallback4);
	ros::Subscriber sub5 = n.subscribe("thrusters/5/input",1000, DebugCallback5);
	ros::Subscriber sub6 = n.subscribe("thrusters/7/input",1000, DebugCallback6);
	ros::Subscriber sub7 = n.subscribe("thrusters/6/input",1000, DebugCallback7);
	if(robot_config == std::string("standard"))
	{
		sub6.shutdown();
		sub7.shutdown();
	}
	ros::spin();
} 
