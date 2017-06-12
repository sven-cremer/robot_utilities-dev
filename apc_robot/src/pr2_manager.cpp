/***********************************************************************************************************************
FILENAME:   pr2_manager.cpp
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

#include <apc_robot/pr2_manager.h>

/***********************************************************************************************************************
controllers
 ***********************************************************************************************************************/
//std::string PR2Manager::RIGHT_ARM_CONTROLLER  	= "r_arm_controller";
//std::string PR2Manager::LEFT_ARM_CONTROLLER   	= "l_arm_controller";
//std::string PR2Manager::PR2_CARTPULL_CONTROLLER = "pr2_cart";

/***********************************************************************************************************************
initializes services and clients
***********************************************************************************************************************/
PR2Manager::PR2Manager(std::string arm_ctrl_new)
{
	// Set controller names
	//arm_controllers_default.push_back("r_arm_controller");
	arm_controllers_default.push_back("l_arm_controller");
	arm_controllers_new.push_back(arm_ctrl_new);

	// ROS service clients
	list_srv_ = nh.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");
	switch_controllers_service = nh.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");

	// Initialize the default controller mode                  // TODO: make this a function (update current controllers)
	for(int i=0;i<arm_controllers_default.size();i++)
	{
		std::string arm_ctrl = arm_controllers_default[i];
		PR2Manager::ControlState state = controllerState(arm_ctrl);

		if(state == RUNNING)
		{
			ROS_INFO("%s is running",arm_ctrl.c_str());
		}
		else if(state == STOPPED)
		{
			ROS_WARN("Turning on %s ...",arm_ctrl.c_str());
			switchControllers(arm_controllers_default,arm_controllers_new);
		}
		else
		{
			ROS_ERROR("The default arm controllers are not running!");
			exit(-1);
		}
	}

	// Stop new arm controller
	for(int i=0;i<arm_controllers_new.size();i++)
	{
		std::string arm_ctrl = arm_controllers_new[i];
		PR2Manager::ControlState state = controllerState(arm_ctrl);

		if(state == RUNNING)
		{
			ROS_WARN("Turning off %s ...",arm_ctrl.c_str());
			switchControllers(arm_controllers_default,arm_controllers_new);
		}
		else if(state == STOPPED)
		{
			ROS_INFO("%s is stopped",arm_ctrl.c_str());
		}
		else
		{
			ROS_WARN("The new arm controllers are not running!");
		}
	}

	// Wait for services
	//ROS_INFO("Waiting for services...");
	//ros::service::waitForService("/pr2_cart/getState",-1);
	//srv_get_State = nh.serviceClient<ice_msgs::getState>("/pr2_cart/getState");


	// Position robot with grippers open
	//robotInit(true);
	defaultTorso = 0.05;
	defaultArmJointsL.clear();
	defaultArmJointsR.clear();

	defaultArmJointsL.push_back(0.744537);	// TODO read from yaml file
	defaultArmJointsL.push_back(0.518209);	// PR2 tactile calibration
	defaultArmJointsL.push_back(1.05854);
	defaultArmJointsL.push_back(-1.80121);
	defaultArmJointsL.push_back(-3.18497);
	defaultArmJointsL.push_back(-1.00517);
	defaultArmJointsL.push_back(2.30901);

	defaultArmJointsR.push_back(-0.263623);
	defaultArmJointsR.push_back(0.856581 );
	defaultArmJointsR.push_back(-0.615513);
	defaultArmJointsR.push_back(-0.960093);
	defaultArmJointsR.push_back(5.0942);
	defaultArmJointsR.push_back(-1.32154);
	defaultArmJointsR.push_back(-3.15487);

	ROS_INFO("PR2Manager initialized!");
}
/***********************************************************************************************************************
Initialize robot position
***********************************************************************************************************************/
void PR2Manager::robotInit(bool open_grippers)
{
	off();	//Make sure manager is off

	// Open grippers
//	if(open_grippers)
//		grippers.open();

	// Lift torso
//	torso.position(0.05);					//To avoid waiting: torso.sendGoal(0.3);    // TODO make this a variable (use 0.3 with cart)
//	ROS_INFO("Torso at %f meters.", 0.01);

	// Position arms
//	arm.sendGoal(leftArmStartPosition());
//	arm.sendGoal(rightArmStartPosition());

//	while(!arm.motionComplete()||!grippers.motionComplete())					// TODO: don't move the arms while the grippers are close
//	{																		//       this could be very bad if the PR2 is grabbing onto something
//		ROS_INFO("Waiting for arm and/or gripper motions to complete ...");
//		sleep(2);
//	}

	torso.sendGoal(defaultTorso);

	arms_joint.sendGoal(defaultArmJointsL, ArmsJoint::LEFT);
	//arms_joint.sendGoal(defaultArmJointsR, ArmsJoint::RIGHT);

	//head.sendGoalCart("torso_frame", 0.48,0.08,-0.05, 3.0);
	head.sendGoal(0.1, 0.5, 0.0, 0.0);

	ROS_INFO("PR2 in position!");

	//Re-initialize controller
	//set_Value_(&srv_reinitCtrl, 0);  TODO do this only when running?

}
/***********************************************************************************************************************
Turn on
***********************************************************************************************************************/
void PR2Manager::on(bool close_grippers)
{
//	if(close_grippers)
//		closeGrippers();
	switchControllers(arm_controllers_new, arm_controllers_default);
}
/***********************************************************************************************************************
Turn off
***********************************************************************************************************************/
void PR2Manager::off(bool open_grippers)
{
	switchControllers(arm_controllers_default, arm_controllers_new);
//	if(open_grippers)
//		openGrippers();
}
/***********************************************************************************************************************
Open/close grippers
***********************************************************************************************************************/
void PR2Manager::openGrippers(PR2Manager::WhichArm a)
{
	switch(a)
	{
	case PR2Manager::LEFT:
		grippers.open(Gripper::LEFT);
		break;
	case PR2Manager::RIGHT:
		grippers.open(Gripper::RIGHT);
		break;
	default:
		grippers.open();
	}

	while(!grippers.motionComplete())
	{
		ROS_INFO("Waiting for gripper motions to complete ...");
		sleep(2);
	}
}
void PR2Manager::closeGrippers(PR2Manager::WhichArm a)
{
	switch(a)
	{
	case PR2Manager::LEFT:
		grippers.close(Gripper::LEFT);
		break;
	case PR2Manager::RIGHT:
		grippers.close(Gripper::RIGHT);
		break;
	default:
		grippers.close();
	}

	while(!grippers.motionComplete())
	{
		ROS_INFO("Waiting for gripper motions to complete ...");
		sleep(2);
	}
}
/***********************************************************************************************************************
SetTorso
***********************************************************************************************************************/
void PR2Manager::setTorso(double height)
{
	torso.sendGoal(height);	// Blocking function
}
/***********************************************************************************************************************
State
***********************************************************************************************************************/
void PR2Manager::printState()
{

}

/***********************************************************************************************************************
clean up clients
***********************************************************************************************************************/
PR2Manager::~PR2Manager()
{
	//off;

}
/***********************************************************************************************************************
Look at Pose
***********************************************************************************************************************/
void PR2Manager::lookAtPoint(geometry_msgs::Point p, double duration)
{
	head.sendGoalCart("torso_lift_link", p, duration);
}
/***********************************************************************************************************************
Goal objects for starting positions
***********************************************************************************************************************/
pr2_controllers_msgs::JointTrajectoryGoal PR2Manager::leftArmStartPosition()
{

	pr2_controllers_msgs::JointTrajectoryGoal goal_left;

	double l_start[7] = {0.149233, 1.16291, 0.159471, -1.71565, -3.1908, -0.521468, -1.52892};

	goal_left.trajectory.joint_names.push_back("l_shoulder_pan_joint");
	goal_left.trajectory.joint_names.push_back("l_shoulder_lift_joint");
	goal_left.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
	goal_left.trajectory.joint_names.push_back("l_elbow_flex_joint");
	goal_left.trajectory.joint_names.push_back("l_forearm_roll_joint");
	goal_left.trajectory.joint_names.push_back("l_wrist_flex_joint");
	goal_left.trajectory.joint_names.push_back("l_wrist_roll_joint");

	// Only one waypoint
	goal_left.trajectory.points.resize(1);

	goal_left.trajectory.points[0].positions.resize(7);
	goal_left.trajectory.points[0].velocities.resize(7);

	for (int i = 0; i<7; ++i)
	{
		goal_left.trajectory.points[0].positions[i] = l_start[i];
		goal_left.trajectory.points[0].velocities[i] = 0.0;
	}

	// To be reached 3 second after starting along the trajectory
	goal_left.trajectory.points[0].time_from_start = ros::Duration(3.0);

	goal_left.trajectory.header.stamp = ros::Time::now();

	return goal_left;
}

pr2_controllers_msgs::JointTrajectoryGoal PR2Manager::rightArmStartPosition()
{
	  pr2_controllers_msgs::JointTrajectoryGoal goal_right;

	  double r_start[7] = {0.0063641, 1.1557, -0.00750675, -1.73534, 3.09916, -0.607375, -1.5531};

	  goal_right.trajectory.joint_names.push_back("r_shoulder_pan_joint");
	  goal_right.trajectory.joint_names.push_back("r_shoulder_lift_joint");
	  goal_right.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
	  goal_right.trajectory.joint_names.push_back("r_elbow_flex_joint");
	  goal_right.trajectory.joint_names.push_back("r_forearm_roll_joint");
	  goal_right.trajectory.joint_names.push_back("r_wrist_flex_joint");
	  goal_right.trajectory.joint_names.push_back("r_wrist_roll_joint");

	  // Only one waypoint
	  goal_right.trajectory.points.resize(1);

	  goal_right.trajectory.points[0].positions.resize(7);
	  goal_right.trajectory.points[0].velocities.resize(7);

	  for (int i = 0; i<7; ++i)
	  {
		  goal_right.trajectory.points[0].positions[i] = r_start[i];
		  goal_right.trajectory.points[0].velocities[i] = 0.0;
	  }

	  // To be reached 3 second after starting along the trajectory
	  goal_right.trajectory.points[0].time_from_start = ros::Duration(3.0);

	  goal_right.trajectory.header.stamp = ros::Time::now();

	  return goal_right;
}
/***********************************************************************************************************************
Get current state of controller
example: http://docs.ros.org/hydro/api/pr2_controller_manager/html/test_8cpp_source.html
 ***********************************************************************************************************************/
PR2Manager::ControlState PR2Manager::controllerState(std::string name)
{
	pr2_mechanism_msgs::ListControllers srv_msg;
	if (!list_srv_.call(srv_msg))
	{
		ROS_WARN("Failed to call list controller service!");
		return FAILURE;
	}
	for (unsigned int i=0; i<srv_msg.response.controllers.size(); i++)
	{
		if (name == srv_msg.response.controllers[i])
		{
			//ROS_INFO(" %s : %s", srv_msg.response.controllers[i].c_str(), srv_msg.response.state[i].c_str());
			if (srv_msg.response.state[i] == "running") return RUNNING;
			else if (srv_msg.response.state[i] == "stopped") return STOPPED;
			else return FAILURE;
		}
	}
	ROS_WARN("Controller not listed!");
	return UNLOADED;
}
/***********************************************************************************************************************
switch controllers
***********************************************************************************************************************/
void PR2Manager::switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers) {

	pr2_mechanism_msgs::SwitchController::Request req;
	pr2_mechanism_msgs::SwitchController::Response res;
	req.start_controllers = start_controllers;
	req.stop_controllers = stop_controllers;
	for(std::vector<std::string>::const_iterator it = start_controllers.begin();
			it != start_controllers.end();
			it++)
	{
		ROS_INFO_STREAM("Trying to start controller " << (*it));
	}
	for(std::vector<std::string>::const_iterator it = stop_controllers.begin();
			it != stop_controllers.end();
			it++)
	{
		ROS_INFO_STREAM("Trying to stop controller " << (*it));
	}
	req.strictness =  pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
	if(!switch_controllers_service.call(req,res))
	{
		ROS_INFO("Call to switch controllers failed entirely");
	}
	if(res.ok != true)
	{
		ROS_INFO("Call to switch controllers reports not ok");
	}
}

bool PR2Manager::setControllers(const std::vector<std::string> default_controllers, const std::vector<std::string> new_controllers)
{
	arm_controllers_default.clear();
	arm_controllers_new.clear();

	arm_controllers_default = default_controllers;
	arm_controllers_new = new_controllers;

	return true;
}
/***********************************************************************************************************************
set functions
***********************************************************************************************************************/
void PR2Manager::setDefaultTorso(double height)
{
	defaultTorso = height;
}
void PR2Manager::setDefaultArmJoints(PR2Manager::WhichArm a, std::vector<double> joint_values)
{
	switch(a)
	{
	case PR2Manager::LEFT:
		defaultArmJointsL = joint_values;
		break;
	case PR2Manager::RIGHT:
		defaultArmJointsR = joint_values;
		break;
	default:
		ROS_WARN("Unable to set arm joint values!");
	}
}
/***********************************************************************************************************************
Main loop for testing
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// Init the ROS node
	ros::init(argc, argv, "pr2_cart_manager");

	ROS_INFO("### Starting pr2_cart_manager ###");

	PR2Manager manager("pr2_cart");

//	ROS_INFO("PRESS [ENTER] TO CLOSE GRIPPERS AND START CONTROLLER");
//	getchar();
//	manager.on();
//
//	ROS_INFO("PRESS [ENTER] TO STOP AND OPEN GRIPPERS");
//	getchar();
//	manager.off();


	ROS_INFO("### Stopping pr2_cart_manager ###");


	ros::shutdown();

	return 0;
}


