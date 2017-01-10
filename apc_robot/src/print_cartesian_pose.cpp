/*
 * print_cartesian_pose.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: Sven Cremer
 */

#include <ros/ros.h>

#include <ros/ros.h>

#include "apc_robot/apc_robot_moveit.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_print_cartesian_pose",ros::init_options::AnonymousName);

	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();

	RobotMoveit robot_moveit;

	std::string tmp;

	geometry_msgs::PoseStamped r_pose;
	geometry_msgs::PoseStamped l_pose;

	while(ros::ok())
	{

		ROS_INFO("Press [enter] to get joint state:");
		std::getline(std::cin, tmp);

		robot_moveit.getEndeffectorPoseStamped(RobotMoveit::RIGHT,&r_pose);
		robot_moveit.getEndeffectorPoseStamped(RobotMoveit::LEFT,&l_pose);

		std::cout<< "Right gripper"<<":\n"
				 << "frame_id: " << r_pose.header.frame_id     << "\n"
				 << "r_pose.position.x     = " << r_pose.pose.position.x     << ";\n"
				 << "r_pose.position.y     = " << r_pose.pose.position.y     << ";\n"
				 << "r_pose.position.z     = " << r_pose.pose.position.z     << ";\n"
				 << "r_pose.orientation.x  = " << r_pose.pose.orientation.x  << ";\n"
				 << "r_pose.orientation.y  = " << r_pose.pose.orientation.y  << ";\n"
				 << "r_pose.orientation.z  = " << r_pose.pose.orientation.z  << ";\n"
				 << "r_pose.orientation.w  = " << r_pose.pose.orientation.w  << ";\n\n";

		std::cout<< "Left gripper:\n"
				 << "frame_id: " << l_pose.header.frame_id     << "\n"
				 << "l_pose.position.x     = " << l_pose.pose.position.x     << ";\n"
				 << "l_pose.position.y     = " << l_pose.pose.position.y     << ";\n"
				 << "l_pose.position.z     = " << l_pose.pose.position.z     << ";\n"
				 << "l_pose.orientation.x  = " << l_pose.pose.orientation.x  << ";\n"
				 << "l_pose.orientation.y  = " << l_pose.pose.orientation.y  << ";\n"
				 << "l_pose.orientation.z  = " << l_pose.pose.orientation.z  << ";\n"
				 << "l_pose.orientation.w  = " << l_pose.pose.orientation.w  << ";\n\n";

	}

	ros::shutdown();

	return 0;
}



