/*
 * testing_node.cpp
 *
 *  Created on: Dec 21, 2014
 *      Author: Sven Cremer
 * 
 * Program used for general testing of apc_baxter classes
 */

//#include <apc_baxter/apc_baxter_commander.h>
#include "apc_robot/apc_robot_moveit.h"


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

  RobotMoveit robot;
//  BaxterCommander baxter_commander;

  /* Test commander */
//  if(!baxter_commander.robotInit())
//  {
//	  ROS_ERROR("main: Robot could not be initialized!");
//  }
//
//  sleep(1.0);
//
//  if(!baxter_commander.disableRobot())
//  {
//	  ROS_ERROR("main: Robot could not be disabled!");
//  }
//  else
//  {
//	  ROS_INFO("Disabled robot!");
//  }
//
//  sleep(1.0);
//
//  if(!baxter_commander.enableRobot())
//  {
//	  ROS_ERROR("main: Robot could not be enabled!");
//  }
//  {
//	  ROS_INFO("Enabled robot!");
//  }

  sleep(0.1);

  /* Move to start pose */
  robot.moveToPose(RobotMoveit::RIGHT,"right_neutral");
  robot.moveToPose(RobotMoveit::LEFT,"left_neutral");

  /* Cartesian path */
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose;
  robot.getEndeffectorPose(RobotMoveit::RIGHT,&target_pose);

  double c = 0.12;

  target_pose.position.x += c;
  target_pose.position.y += c;
  target_pose.position.z -= c;
  waypoints.push_back(target_pose);

  target_pose.position.y += 3*c;
  waypoints.push_back(target_pose);

  target_pose.position.x -= c;
  target_pose.position.y -= c;
  target_pose.position.z += c;
  waypoints.push_back(target_pose);

  target_pose.position.y -= 3*c;
  waypoints.push_back(target_pose);

  robot.executeCartePath(RobotMoveit::RIGHT, waypoints);

//  ROS_INFO("Done!");
//  ros::shutdown();
//
//  return 0;

  /* Goal 1 for right arm */
  std::vector<double> joint_1;					// Joint position

  joint_1.push_back(-0.9808801953);
  joint_1.push_back(-0.4099889011);
  joint_1.push_back(2.3575322526);
  joint_1.push_back(1.4034062902);
  joint_1.push_back(-1.2265578818);
  joint_1.push_back(0.7220344007);
  joint_1.push_back(0.0247018696);

  geometry_msgs::Pose target_1;					// End-effector position (/right_gripper)
  target_1.position.x    = 0.562572067614  ;
  target_1.position.y    = -0.66661394961  ;
  target_1.position.z    = 0.86364619136   ;
  target_1.orientation.x = 0.0132978988463 ;
  target_1.orientation.y = 0.684069917849  ;
  target_1.orientation.z = -0.0323280582007;
  target_1.orientation.w = 0.72857834859   ;

  /* Goal 2 for right arm */
  std::vector<double> joint_2;					// Joint position

  joint_2.push_back(0.2411131029);
  joint_2.push_back(-0.7120716262);
  joint_2.push_back(0.8446270724);
  joint_2.push_back(2.0868783869);
  joint_2.push_back(0.0293828019);
  joint_2.push_back(-1.2447985456);
  joint_2.push_back(-0.6427862872);

  geometry_msgs::Pose target_2;					// End-effector position
  target_2.position.x    = 0.783441506241  ;
  target_2.position.y    = -0.193245585037 ;
  target_2.position.z    = 0.327348973328  ;
  target_2.orientation.x = -0.0190489300163 ;
  target_2.orientation.y = 0.691004875435   ;
  target_2.orientation.z = -0.00119723545227;
  target_2.orientation.w = 0.722598067405   ;

  /* Display Joint positions */

  ROS_INFO("Starting pose:");
  robot.printJointStates();

  /* Execute Joint goals */

  robot.executeJointGoal(RobotMoveit::RIGHT, joint_1);

  ROS_INFO("->Reached Joint goal 1");
  robot.printJointStates();
  sleep(0.3);

  robot.executeJointGoal(RobotMoveit::RIGHT, joint_2);

  ROS_INFO("->Reached Joint goal 2");
  robot.printJointStates();
  sleep(0.3);


  /* Execute Cartesian goals */

  robot.executeCarteGoal(RobotMoveit::RIGHT, target_1);

  ROS_INFO("->Reached Cartesian goal 1");
  sleep(0.3);

  robot.executeCarteGoalWithConstraint(RobotMoveit::RIGHT, target_2);

  ROS_INFO("->Reached Cartesian goal 2");
  sleep(0.3);


  ROS_INFO("Done!");
  ros::shutdown();

  return 0;
}
