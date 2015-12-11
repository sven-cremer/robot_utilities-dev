/*
 * test_video_demo.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: sven
 * 
 * Testing for video submission
 * 
 */

#include <apc_baxter/apc_baxter_commander.h>
#include "../../include/apc_baxter/apc_robot_grippers.h"
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

  std::string tmp;

  BaxterCommander baxter_commander;
  RobotMoveit baxter_moveit;
  Gripper baxter_grippers;

  bool motion_complete = true;

  /* Init robot */
  if(!baxter_commander.robotInit())
  {
	  ROS_ERROR("main: Robot could not be initialized!");
  }

  sleep(1.0);

//  if(!baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_grab_kinect"))
//  {
//	  motion_complete = false;
//  }
//  if(!baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_neutral"))
//  {
//	  motion_complete = false;
//  }
//
//  /* Open grippers */
//  baxter_grippers.open(Gripper::BOTH);
//
//  /* Grab kinect */
//  ROS_INFO("Put kinect in right arm and press [enter]");
//  std::getline(std::cin, tmp);
//  baxter_grippers.close(Gripper::RIGHT);

  /* Initialize */
  baxter_grippers.open(Gripper::LEFT);
  sleep(1.0);
  baxter_grippers.close(Gripper::LEFT);

  if(!baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_neutral"))
  {
	  motion_complete = false;
  }
  if(!baxter_moveit.moveToPose(RobotMoveit::RIGHT,"right_p01_neutral"))
  {
	  motion_complete = false;
  }

  sleep(1.0);

  /* Move right */
  baxter_moveit.moveToPose(RobotMoveit::RIGHT,"right_p02_camera_pose");//,false);
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p01_ready");//,false);

  //sleep(4.0);

  /* Grab object */
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p02_approach");
  baxter_grippers.open(Gripper::LEFT);
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p03_approach");
  sleep(0.5);
  baxter_grippers.closeGently(Gripper::LEFT);

  sleep(0.5);

  /* Remove object */
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p04_lift");

  baxter_moveit.moveToPose(RobotMoveit::RIGHT,"right_p02_bins");//,false);
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p05_neutral");//,false);

  //sleep(4.0);

  /* Place in bin object */
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p06_above_bin");
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p07_bin");
  baxter_grippers.open(Gripper::LEFT);

  /* Return */
  baxter_moveit.moveToPose(RobotMoveit::LEFT,"left_p08_clear_bin");

  baxter_moveit.moveToPose(RobotMoveit::RIGHT,"right_p01_neutral");//,false);
  //baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_neutral");//,false);

  sleep(1.0);


  ROS_INFO("Done!");
  ros::shutdown();

  return 0;


//  /* Grab object */
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_move_in_front_of_shelf");
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_grab");
//  baxter_grippers.close(Gripper::LEFT);
//
//  sleep(1.0);
//
//  /* Place object in the bin */
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_move_in_front_of_shelf");
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_above_box");
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_inside_box");
//  baxter_grippers.open(Gripper::LEFT);
//
//  sleep(1.0);
//
//  /* Return to neutral */
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_above_box");
//  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_neutral");
//
//  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_grab_kinect");
//
//  baxter_grippers.open(Gripper::RIGHT);
//
//  sleep(1.0);
//
//  ROS_INFO("Done!");
//  ros::shutdown();
//
//  return 0;
}



