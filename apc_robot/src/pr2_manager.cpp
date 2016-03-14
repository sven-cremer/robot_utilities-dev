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
2016.02.17  SC     changed to a general PR2manager
***********************************************************************************************************************/

#include <apc_robot/pr2_manager.h>

/***********************************************************************************************************************
controllers
 ***********************************************************************************************************************/
std::string PR2Manager::RIGHT_ARM_CONTROLLER  	= "r_arm_controller";
std::string PR2Manager::LEFT_ARM_CONTROLLER   	= "l_arm_controller";
std::string PR2Manager::PR2_CARTPULL_CONTROLLER = "pr2_cart";

/***********************************************************************************************************************
initializes services and clients
***********************************************************************************************************************/
PR2Manager::PR2Manager()
{
	list_srv_ = nh.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");

	// Switch client for cart pushing
	switch_controllers_service = nh.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
	arm_controllers_default.push_back(LEFT_ARM_CONTROLLER);
	arm_controllers_default.push_back(RIGHT_ARM_CONTROLLER);
	arm_controllers_cart.push_back(PR2_CARTPULL_CONTROLLER);


	// Initialize the current Controller modes                  // TODO: make this a function (update current controllers)
	if(controllerState(LEFT_ARM_CONTROLLER) == RUNNING)
	{
		ROS_INFO("Left arm in POSITION_CONTROL");
	}
	else if(controllerState(LEFT_ARM_CONTROLLER) == STOPPED)
	{
		ROS_WARN("Turning on arm controllers ...");
		switchControllers(arm_controllers_default,arm_controllers_cart);
	}
	else
	{
		ROS_ERROR("No left arm controller running!");
		exit(-1);
	}

	if(controllerState(RIGHT_ARM_CONTROLLER) == RUNNING)
	{
		ROS_INFO("Right arm in POSITION_CONTROL");
	}
	else if(controllerState(RIGHT_ARM_CONTROLLER) == STOPPED)
	{
		ROS_WARN("Turning on arm controllers ...");
		switchControllers(arm_controllers_default,arm_controllers_cart);
	}
	else
	{
		ROS_ERROR("No right arm controller running!");
		exit(-1);
	}

	if(controllerState(PR2_CARTPULL_CONTROLLER) == RUNNING)
	{
		ROS_WARN("Turning off cart controller");
		switchControllers(arm_controllers_default,arm_controllers_cart);
	}
	else if(controllerState(PR2_CARTPULL_CONTROLLER) == STOPPED)
	{
		ROS_INFO("Cart controller is stopped ...");
	}
	else
	{
		ROS_ERROR("No cart controller running!");
		exit(-1);
	}


	  // Wait for services
	  ROS_INFO("Waiting for services...");

	  ros::service::waitForService("/pr2_cart/getState",-1);

	  srv_get_State = nh.serviceClient<ice_msgs::getState>("/pr2_cart/getState");


//	  ros::service::waitForService("/pr2_cart/reinitCtrl",-1);
//	  srv_reinitCtrl = nh.serviceClient<ice_msgs::setValue>("/pr2_cart/reinitCtrl");

		// Position robot with grippers open
		robotInit(true);


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

	torso.sendGoal(0.2);


	std::vector<double> l_joints;
	l_joints.push_back(0.149233);
	l_joints.push_back(1.16291);
	l_joints.push_back(0.159471);
	l_joints.push_back(-1.71565);
	l_joints.push_back(-3.1908);
	l_joints.push_back(-0.521468);
	l_joints.push_back(-1.52892);

	std::vector<double> r_joints;
	r_joints.push_back(0.0063641);
	r_joints.push_back(1.1557);
	r_joints.push_back(-0.00750675);
	r_joints.push_back(-1.73534);
	r_joints.push_back(3.09916);
	r_joints.push_back(-0.607375);
	r_joints.push_back(-1.5531);

	arms.sendGoal(l_joints, ArmsJoint::LEFT);
	arms.sendGoal(r_joints, ArmsJoint::RIGHT);

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
	switchControllers(arm_controllers_cart, arm_controllers_default);
}
/***********************************************************************************************************************
Turn off
***********************************************************************************************************************/
void PR2Manager::off(bool open_grippers)
{
	switchControllers(arm_controllers_default, arm_controllers_cart);
//	if(open_grippers)
//		openGrippers();
}
/***********************************************************************************************************************
Open/close grippers
***********************************************************************************************************************/
void PR2Manager::openGrippers()
{
	grippers.open();
	while(!grippers.motionComplete())
	{
		ROS_INFO("Waiting for gripper motions to complete ...");
		sleep(2);
	}
}
void PR2Manager::closeGrippers()
{
	grippers.close();
	while(!grippers.motionComplete())
	{
		ROS_INFO("Waiting for gripper motions to complete ...");
		sleep(2);
	}
}

/***********************************************************************************************************************
State
***********************************************************************************************************************/
bool PR2Manager::get_State_(ice_msgs::getState * currentState)
{
	ice_msgs::getState temp;

	if (srv_get_State.call(temp))
	{
		currentState->response = temp.response;
		return true;
	}
	{
		ROS_ERROR("Failed to get state!");
		return false;
	}
}

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


/***********************************************************************************************************************
Main loop for testing
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// Init the ROS node
	ros::init(argc, argv, "pr2_cart_manager");

	ROS_INFO("### Starting pr2_cart_manager ###");

	PR2Manager manager;

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


