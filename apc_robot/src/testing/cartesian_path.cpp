/*
 * cartesian_path.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: sven
 * 
 * Gripper moves in a rectangular path specified by Cartesian coordinates.
 * Made for Baxter drawing test.
 */


#include <apc_baxter/apc_baxter_commander.h>
#include "../../include/apc_baxter/apc_robot_moveit.h"


/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, demonstrates use of BaxterMoveit class
***********************************************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apc_testing_node",ros::init_options::AnonymousName);
//  ros::NodeHandle n;

  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  RobotMoveit baxter_moveit;
  BaxterCommander baxter_commander;


  if(!baxter_commander.enableRobot())
  {
	  ROS_ERROR("main: Robot could not be enabled!");
  }
  {
	  ROS_INFO("Enabled robot!");
  }

  sleep(0.1);

//  /* Move to neutral pose */
//  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_neutral");
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_neutral");

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose;

  // Move into position
  target_pose.position.x     = 0.55;
  target_pose.position.y     = -0.35;
  target_pose.position.z     = 0.10;
  target_pose.orientation.x  = 0.0;
  target_pose.orientation.y  = -1.0;
  target_pose.orientation.z  = 0.0;
  target_pose.orientation.w  = 0.0;

  baxter_moveit.executeCarteGoal(RobotMoveit::RIGHT, target_pose);

  /* Cartesian path */
  int N = 8;
  double h_close = 0.08;
  double h_paper = 0.014;
  double length  = 0.10;
  double width   = 0.15;

  std::cout<<"Height: "<< h_paper <<"\n";

  // Get starting position
  baxter_moveit.getEndeffectorPose(RobotMoveit::RIGHT,&target_pose);
  waypoints.push_back(target_pose);

  // bottom-right
  target_pose.position.z = h_close;
  waypoints.push_back(target_pose);

  // bottom-right (touch paper)
  target_pose.position.z = h_paper;
  for(int i=0;i<N;i++)
    waypoints.push_back(target_pose);

  for(int loops=0;loops<2;loops++)
  {

	  // bottom-left
	  target_pose.position.y     += length;
	  for(int i=0;i<N;i++)
		  waypoints.push_back(target_pose);

	  // top-left
	  target_pose.position.x     += width;
	  for(int i=0;i<N;i++)
		  waypoints.push_back(target_pose);

	  // top-right
	  target_pose.position.y     -= length;
	  for(int i=0;i<N;i++)
		  waypoints.push_back(target_pose);

	  // bottom-right
	  target_pose.position.x     -= width;
	  for(int i=0;i<N;i++)
		  waypoints.push_back(target_pose);
  }

  // move up
  target_pose.position.z = h_close;
  waypoints.push_back(target_pose);

  baxter_moveit.executeCartePath(RobotMoveit::RIGHT, waypoints);

  ROS_INFO("Done!");
  ros::shutdown();

  return 0;

}


