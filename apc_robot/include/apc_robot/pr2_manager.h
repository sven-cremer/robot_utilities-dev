/***********************************************************************************************************************
FILENAME:   pr2_manager.h
AUTHORS:    Sven Cremer 		sven.cremer@mavs.uta.edu
            University of Texas at Arlington, Copyright (C) 2016.

DESCRIPTION:
Manger for pr2 realtime controller. Uses custom pr2_motion_clients packages for torso, arm, and gripper setup.

PUBLISHES:  NA
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2014.02.07  SC     original file creation
2014.02.14  SC     code cleanup, new gripper client
2014.02.17  SC     services for setting gains
2015.10.26  SC     ported to hydro
2016.02.17  SC     changed to a general PR2manager
***********************************************************************************************************************/

#ifndef PR2_MANAGER_H_
#define PR2_MANAGER_H_



#include <ros/ros.h>
#include <ros/package.h>

// Motion clients
#include <apc_robot/apc_arms_joint.h>
#include <apc_robot/apc_arms_cartesian.h>
#include <apc_robot/pr2_torso.h>
#include <apc_robot/apc_robot_grippers.h>
#include <apc_robot/pr2_head.h>

// Service messages
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>


class PR2Manager
{
public:

	  enum ControlState{
	    RUNNING,
	    STOPPED,
	    UNLOADED,
	    FAILURE
	  };

	  enum WhichArm{
	    BOTH,
	    LEFT,
	    RIGHT
	  };

	PR2Manager(std::string arm_ctrl_new = "");
	~PR2Manager();

	void robotInit(	bool open_grippers=true);
	void on(		bool close_grippers=true);
	void off(		bool open_grippers=true);

	void setTorso(double height);

	void openGrippers(PR2Manager::WhichArm a = PR2Manager::BOTH);
	void closeGrippers(PR2Manager::WhichArm a = PR2Manager::BOTH);

	void printState();

	void switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers);
	PR2Manager::ControlState controllerState(std::string name);

	bool setControllers(const std::vector<std::string> default_controllers, const std::vector<std::string> new_controllers);

	void setDefaultTorso(double height);
	void setDefaultArmJoints(PR2Manager::WhichArm a, std::vector<double> joint_values);

private:

	ros::NodeHandle nh;
	ros::ServiceClient switch_controllers_service;
	ros::ServiceClient list_srv_;

	std::vector<std::string> arm_controllers_default;
	std::vector<std::string> arm_controllers_new;

	Torso torso;
	Gripper grippers;
	ArmsJoint arms_joint;
	//ArmsCartesian arms_cart;
	Head head;

	double defaultTorso;
	std::vector<double> defaultArmJointsL;
	std::vector<double> defaultArmJointsR;

	pr2_controllers_msgs::JointTrajectoryGoal leftArmStartPosition();
	pr2_controllers_msgs::JointTrajectoryGoal rightArmStartPosition();

//	static std::string RIGHT_ARM_CONTROLLER;
//	static std::string LEFT_ARM_CONTROLLER;
//	static std::string PR2_CARTPULL_CONTROLLER;

//	ros::ServiceClient srv_reinitCtrl;

};





#endif /* PR2_MANAGER_H_ */
