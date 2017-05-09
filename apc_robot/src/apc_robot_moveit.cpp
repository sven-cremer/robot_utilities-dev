/*!
 * 	\name 	   RobotMoveit
 *  \brief     Controls PR2 arm movements through MoveIt
 *  \details   Arm movements are implmented through the MoveIt API
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Dec 14, 2014
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#include "apc_robot/apc_robot_moveit.h"

RobotMoveit::RobotMoveit()
{
	ROS_INFO("RobotMoveit::RobotMoveit - Initializing");

	// Planning scene interface
	planning_scene_diff_pub = nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 10);
	pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	pub_pick = nh.advertise<moveit_msgs::PickupActionGoal>("/pickup/goal", 10);

	// Load robot name from parameter server
	std::string robot = "pr2";
	if(!(nh.getParam("/apc_robot/robot", robot)))
		ROS_WARN("Robot name not found! Using defaulft (%s)!", robot.c_str());

	// Load joint states topic name
	std::string param_joint_states  = "/apc_robot/"+robot+"/joint_states";
	joint_states = "/joint_states";
	if(!(nh.getParam(param_joint_states,  joint_states)))
		ROS_WARN("Name for joint states not found! Using defaulft (%s)!", joint_states.c_str());

	// Load move group names
	std::string param_left  = "/apc_robot/"+robot+"/move_group_left";
	std::string param_right = "/apc_robot/"+robot+"/move_group_right";
	move_group_left  = "left_arm";
	move_group_right = "right_arm";

	if(!(nh.getParam(param_left,  move_group_left)))
		ROS_WARN("Name for left move group not found! Using defaulft (%s)!", move_group_left.c_str());

	if(!(nh.getParam(param_right, move_group_right)))
		ROS_WARN("Name for right move group not found! Using defaulft (%s)!", move_group_right.c_str());

	// Kinematics
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model::RobotModelPtr(robot_model_loader.getModel());

	kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();

	// Get arm names
	joint_names_right_arm = kinematic_state->getJointModelGroup(move_group_right)->getActiveJointModelNames();
	joint_names_left_arm  = kinematic_state->getJointModelGroup(move_group_left)->getActiveJointModelNames();
	
	// Move Group Interface
	group_left_arm  = new moveit::planning_interface::MoveGroup(move_group_left);		// TODO Gets stuck here without move_group.launch
	group_right_arm = new moveit::planning_interface::MoveGroup(move_group_right);

	// Set planners
	group_left_arm->setPlannerId("RRTConnectkConfigDefault");
	group_left_arm->setPlanningTime(15);

	group_right_arm->setPlannerId("RRTConnectkConfigDefault");
	group_right_arm->setPlanningTime(15);

	// Set tolerances
//	double tol = 0.10;
//	group_left_arm ->setGoalOrientationTolerance(tol);
//	group_right_arm->setGoalOrientationTolerance(tol);
//	group_right_arm->setGoalJointTolerance(tol);
//	group_right_arm->setGoalPositionTolerance(tol);
//	group_right_arm->setGoalTolerance(tol);

	joint_pos_right_arm.resize(joint_names_right_arm.size());
	joint_pos_left_arm.resize(joint_names_left_arm.size());

	leftMotionInProgress = false;
	rightMotionInProgress = false;

	// Check if joint_states is being published
	ROS_INFO("Checking if %s is publishing...",joint_states.c_str());
	sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states,ros::Duration(1,0));
	if (msg == NULL)
		ROS_ERROR("Could not detect a %s topic!",joint_states.c_str());


	ROS_INFO("*** Initialized RobotMoveit class ***");
}

RobotMoveit::~RobotMoveit()
{
	if(group_left_arm)
		delete group_left_arm;
	if(group_right_arm)
		delete group_right_arm;
}


bool RobotMoveit::pickObject(std::string name, geometry_msgs::PoseStamped p)
{
	// Get model path
	std::string filePath;
	std::string paramName = "/apc/" + name + "_model_path";
	if (!nh.getParam(paramName, filePath)) {
		ROS_ERROR_STREAM("Could not find model path for " << name);
		return false;
	}

	// Load shape
	shape_msgs::Mesh meshRosMessage;
	shapes::ShapeMsg meshMessage = meshRosMessage;
	std::ifstream fin(filePath.c_str());
	bool couldLoadShape = false;
	if (fin) {
		// get length of the file:
		fin.seekg(0, fin.end);
		int length = fin.tellg();
		fin.seekg(0, fin.beg);
		char *buffer = new char[length];
		// read data
		fin.read(buffer, length);
		fin.close();
		shapes::Mesh *pShelfMesh =
				shapes::createMeshFromBinary(buffer, length, filePath);
		delete buffer;
		if (pShelfMesh) {
			couldLoadShape = shapes::constructMsgFromShape(pShelfMesh, meshMessage);
			delete pShelfMesh;
		}
	}

	if (!couldLoadShape) {
		ROS_ERROR_STREAM("Could not load mesh for object " << name);
		return false;
	}

	// 1) Table collision object
	moveit_msgs::CollisionObject coTable;
	coTable.id = "table";
	coTable.header.stamp = ros::Time::now();
	coTable.header.frame_id = p.header.frame_id;

	// remove table
	coTable.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(coTable);

	// add table
	coTable.operation = moveit_msgs::CollisionObject::ADD;
	coTable.primitives.resize(1);
	coTable.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	coTable.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	coTable.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.5;
	coTable.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.8;
	coTable.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
	coTable.primitive_poses.resize(1);
	coTable.primitive_poses[0].position.x = 0.0;
	coTable.primitive_poses[0].position.y = 3.54;
	coTable.primitive_poses[0].position.z = 0.55-0.03;//-0.15;
	coTable.primitive_poses[0].orientation.w = 1.0;
	pub_co.publish(coTable);

	// 2) Coke Can collision object
	moveit_msgs::CollisionObject coObject;
	coObject.id = name;
	coObject.header.stamp = ros::Time::now();
	coObject.header.frame_id = p.header.frame_id;

	// remove objects
	coObject.operation = coObject.REMOVE;
	pub_co.publish(coObject);

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.object = coObject;
	pub_aco.publish(attached_object);

	// add object
	coObject.operation = coObject.ADD;
	coObject.meshes.push_back(boost::get<shape_msgs::Mesh>(meshMessage));
	geometry_msgs::Pose p2 = p.pose;
	p2.position.z -= 0.01;
	coObject.mesh_poses.push_back(p2);
	/*
	coObject.primitives.resize(1);
	coObject.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	coObject.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	coObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.11;
	coObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.11;
	coObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.14;
	coObject.primitive_poses.push_back(p.pose);
//	coObject.primitive_poses[0].position.x += 0.05;
//	coObject.primitive_poses[0].position.y += 0.05;
	coObject.primitive_poses[0].position.z += 0.06;
	*/
	pub_co.publish(coObject);

	// Attach object
//	attached_object.link_name = "r_wrist_roll_link";
//	attached_object.object = coObject;
//	pub_co.publish(coObject);

	// Publish planning scene
	moveit_msgs::PlanningScene planning_scene;
//	planning_scene.world.collision_objects.push_back(coTable);
//	planning_scene.world.collision_objects.push_back(coObject);
//	//planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
	planning_scene.is_diff = true;
//	planning_scene_diff_pub.publish(planning_scene);

	// Allow planning scene to initialize
	ros::Duration(3.0).sleep();

	// Plan grasp
	std::vector<moveit_msgs::Grasp> grasps;
	moveit_msgs::Grasp g;

	// Pose in front of object consistent with approach direction
	g.grasp_pose = p;
	g.grasp_pose.pose.orientation.w =  0;		// Gripper pointing left w.r.t. map frame
	g.grasp_pose.pose.orientation.z =  1;
	g.grasp_pose.pose.position.x += 0.15;
	g.grasp_pose.pose.position.y += 0.035;		// align with gripper center
	g.grasp_pose.pose.position.z += 0.075;		// pick middle of object

	// Move forward w.r.t. wrist
	g.pre_grasp_approach.direction.vector.x = 1.0;
	g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
	g.pre_grasp_approach.min_distance = 0.08;
	g.pre_grasp_approach.desired_distance = 0.12;

	// Lift object
	g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
	g.post_grasp_retreat.direction.vector.z = 1.0;
	g.post_grasp_retreat.min_distance = 0.05;
	g.post_grasp_retreat.desired_distance = 0.20;

	// Gripper open and close joint positions
	openGripper(g.pre_grasp_posture);
	closedGripper(g.grasp_posture);
/*
	// Only works on the real PR2:
	  g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
	  g.pre_grasp_posture.points.resize(1);
	  g.pre_grasp_posture.points[0].positions.resize(1);
	  g.pre_grasp_posture.points[0].positions[0] = 0.95;

	  g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
	  g.grasp_posture.points.resize(1);
	  g.grasp_posture.points[0].positions.resize(1);
	  g.grasp_posture.points[0].positions[0] = 0.05;
*/

//	g.allowed_touch_objects.push_back("r_forearm_link");
//	g.allowed_touch_objects.push_back(name);

	 // TODO add path constraints

	grasps.push_back(g);

	//std::cout<<g<<"\n---\n";

	ROS_INFO("Attempting grasp ... ");

	moveit_msgs::PickupActionGoal ag;
	ag.header.stamp = ros::Time::now();
	//ag.header.frame_id = "map"; // ?
	ag.goal_id.stamp = ag.header.stamp;
	//ag.goal_id.id = "right_arm_pick_place";	// + timestamps
	ag.goal.target_name = name;
	ag.goal.group_name = "right_arm";
	//ag.goal.end_effector = "right_gripper";  // ?
	ag.goal.possible_grasps.push_back(g);
	ag.goal.support_surface_name ="table";
	ag.goal.allow_gripper_support_collision = true;
	ag.goal.minimize_object_distance = false;
	ag.goal.allowed_planning_time = 45;
	//ag.goal.allowed_touch_objects.push_back("all");

	ag.goal.planning_options.planning_scene_diff.is_diff = true;
	ag.goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
	ag.goal.planning_options.plan_only = false;
	ag.goal.planning_options.look_around = false;
	ag.goal.planning_options.look_around_attempts = 0;
	ag.goal.planning_options.max_safe_execution_cost = 0.0;
	ag.goal.planning_options.replan = true;
	ag.goal.planning_options.replan_attempts = 3;
	ag.goal.planning_options.replan_delay = 2.0;

	ag.goal.attached_object_touch_links.push_back("r_elbow_flex_link"          );
	ag.goal.attached_object_touch_links.push_back("r_forearm_link"             );
	ag.goal.attached_object_touch_links.push_back("r_wrist_flex_link"          );
	ag.goal.attached_object_touch_links.push_back("r_wrist_roll_link"          );

	ag.goal.attached_object_touch_links.push_back("r_gripper_l_finger_link"     );
    ag.goal.attached_object_touch_links.push_back("r_gripper_palm_link"         );
    ag.goal.attached_object_touch_links.push_back("r_force_torque_link"         );
    ag.goal.attached_object_touch_links.push_back("r_gripper_motor_accelerometer_link" );
    ag.goal.attached_object_touch_links.push_back("r_gripper_motor_slider_link" );
    ag.goal.attached_object_touch_links.push_back("r_gripper_motor_screw_link"  );
    ag.goal.attached_object_touch_links.push_back("r_gripper_l_finger_tip_link" );
    ag.goal.attached_object_touch_links.push_back("r_gripper_r_finger_link"     );
    ag.goal.attached_object_touch_links.push_back("r_gripper_r_finger_tip_link" );

    //ag.goal.attached_object_touch_links.push_back("table");

    std::cout<<ag<<"\n---\n";

    // A) Send command directly to pick action server
	pub_pick.publish(ag);
	return true;

	// B) Use move_group interface
	group_right_arm->setSupportSurfaceName("table");
	group_right_arm->setPlanningTime(30.0);
	//group_right_arm->setNumPlanningAttempts(50);
	group_right_arm->setPlannerId("LBKPIECE1");		// RRTConnectkConfigDefault
	std::string tmp = "";
	//group_right_arm->attachObject(name,tmp,ag.goal.attached_object_touch_links); // Solves " Found a contact between 'table' (type 'Object') and 'r_forearm_link' (type 'Robot link'), which constitutes a collision."
	try
	{
		std::cout<<"Picking...\n";
		group_right_arm->pick(name, grasps);
		//group_right_arm->planGraspsAndPick(name);
	}
	catch (moveit::planning_interface::MoveItErrorCode &e)
	{
		ROS_ERROR("->Could not move arm because: %s", e);
		return false;
	}

	std::cout<<"Done!\n";
	return true;

	// C) Other testing:

	// Add to environment
	ROS_INFO("Adding object to environment");
	//moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(coTable);
	planning_scene.world.collision_objects.push_back(coObject);
	planning_scene.is_diff = true;
	planning_scene_diff_pub.publish(planning_scene);

	// Attach object to robot
	ROS_INFO("Attaching object to robot");

	/* First, define the REMOVE object message*/
	moveit_msgs::CollisionObject remove_object;
	remove_object.id = name;
	remove_object.header.frame_id = p.header.frame_id;
	remove_object.operation = remove_object.REMOVE;

	/* Carry out the REMOVE + ATTACH operation */
	geometry_msgs::Pose pz;
	pz.position.z = -0.04;
	pz.orientation.w=1;
	//moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = "r_wrist_roll_link";
//	attached_object.object.id = name;
//	attached_object.object.header.frame_id = "r_wrist_roll_link";
//	attached_object.object.meshes.push_back(boost::get<shape_msgs::Mesh>(meshMessage));
//	attached_object.object.mesh_poses.push_back(pz);
//	attached_object.object.operation = attached_object.object.ADD;
	attached_object.object = coObject;

	planning_scene.world.collision_objects.clear();
	planning_scene.world.collision_objects.push_back(remove_object);
	planning_scene.robot_state.attached_collision_objects.clear();
	planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
	planning_scene.is_diff = true;
	planning_scene_diff_pub.publish(planning_scene);

	sleep(10.0);

	// Detaching object from robot
	ROS_INFO("Detaching object from robot");

	/* First, define the DETACH object message*/
	moveit_msgs::AttachedCollisionObject detach_object;
	detach_object.object.id = attached_object.object.id;
	detach_object.link_name = attached_object.link_name;
	detach_object.object.operation = attached_object.object.REMOVE;

	/* Carry out the DETACH + ADD operation */
	planning_scene.robot_state.attached_collision_objects.clear();
	planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
	planning_scene.world.collision_objects.clear();
	planning_scene.world.collision_objects.push_back(coObject);
	planning_scene_diff_pub.publish(planning_scene);

	sleep(15.0);

	// Remove object
	ROS_INFO("Removing the object from the world.");
	planning_scene.robot_state.attached_collision_objects.clear();
	planning_scene.world.collision_objects.clear();
	planning_scene.world.collision_objects.push_back(remove_object);
	planning_scene_diff_pub.publish(planning_scene);

	return true;
}

void RobotMoveit::openGripper(trajectory_msgs::JointTrajectory& posture){
  posture.joint_names.resize(6);
  posture.joint_names[0] = "r_gripper_joint";
  posture.joint_names[1] = "r_gripper_motor_screw_joint";
  posture.joint_names[2] = "r_gripper_l_finger_joint";
  posture.joint_names[3] = "r_gripper_r_finger_joint";
  posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 1.0;
  posture.points[0].positions[1] = 1.0;
  posture.points[0].positions[2] = 0.477;
  posture.points[0].positions[3] = 0.477;
  posture.points[0].positions[4] = 0.477;
  posture.points[0].positions[5] = 0.477;
}

void RobotMoveit::closedGripper(trajectory_msgs::JointTrajectory& posture){
  posture.joint_names.resize(6);
  posture.joint_names[0] = "r_gripper_joint";
  posture.joint_names[1] = "r_gripper_motor_screw_joint";
  posture.joint_names[2] = "r_gripper_l_finger_joint";
  posture.joint_names[3] = "r_gripper_r_finger_joint";
  posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;
  posture.points[0].positions[2] = 0.002;
  posture.points[0].positions[3] = 0.002;
  posture.points[0].positions[4] = 0.002;
  posture.points[0].positions[5] = 0.002;
}

bool RobotMoveit::updateJointStates()	// This function is meant to be called occasionally (it is not optimized)
{

	joint_pos_right_arm = group_right_arm->getCurrentJointValues();
	joint_pos_left_arm  = group_left_arm->getCurrentJointValues();

	/*
	sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/robot/joint_states",ros::Duration(1,0));

	if (msg == NULL)
	{
		ROS_ERROR("No robot joint state message received!");
		return false;
	}

	// Update right arm
	for(unsigned int i = 0; i < joint_names_right_arm.size(); i++)
	{
		for (int k = 0; k < msg->name.size(); k++)
		{
			if (joint_names_right_arm[i] == msg->name[k])
			{
				joint_pos_right_arm[i] = msg->position[k]; 		// Update joint position
				k = msg->name.size();
			}
		}
	}

	// Update left arm
	for(unsigned int i = 0; i < joint_names_left_arm.size(); i++)
	{
		for (int k = 0; k < msg->name.size(); k++)
		{
			if (joint_names_left_arm[i] == msg->name[k])
			{
				joint_pos_left_arm[i] = msg->position[k]; 		// Update joint position
				k = msg->name.size();
			}
		}
	}
	*/
	return true;
}

void RobotMoveit::printJointStates()
{

	updateJointStates();

	for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
	{
		ROS_INFO(" Joint %s: %f", joint_names_right_arm[i].c_str(), joint_pos_right_arm[i]);
	}

	for(std::size_t i = 0; i < joint_names_left_arm.size(); ++i)
	{
		ROS_INFO(" Joint %s: %f", joint_names_left_arm[i].c_str(), joint_pos_left_arm[i]);
	}

}

void RobotMoveit::printJointStatesSRDF()
{

	updateJointStates();

	// Print right arm postion
	std::cout << "    <group_state name=\"right_\" group=\"right_arm\">\n";
	for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
	{
		std::cout << "        <joint name=\"" << joint_names_right_arm[i].c_str() <<"\" value=\"" << joint_pos_right_arm[i] << "\" />\n";
	}
	std::cout << "    </group_state>\n\n";

	// Print left arm postion
	std::cout << "    <group_state name=\"left_\" group=\"left_arm\">\n";
	for(std::size_t i = 0; i < joint_names_left_arm.size(); ++i)
	{
		std::cout << "        <joint name=\"" << joint_names_left_arm[i].c_str() <<"\" value=\"" << joint_pos_left_arm[i] << "\" />\n";
	}
	std::cout << "    </group_state>\n\n";

}

bool RobotMoveit::getEndeffectorPose(RobotMoveit::WhichArm arm, geometry_msgs::Pose* current_pose)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	*current_pose = group->getCurrentPose().pose;

	return true;
}

bool RobotMoveit::getEndeffectorPoseStamped(RobotMoveit::WhichArm arm, geometry_msgs::PoseStamped* current_pose)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	*current_pose = group->getCurrentPose();

	return true;
}


bool RobotMoveit::selectArmGroup(RobotMoveit::WhichArm arm, moveit::planning_interface::MoveGroup** group)
{
	switch(arm)
	{
	case RobotMoveit::LEFT:
		*group = group_left_arm;
		break;
	case RobotMoveit::RIGHT:
		*group = group_right_arm;
		break;
	case RobotMoveit::BOTH:
		//group = group_both_arms;
		//break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}

	return true;

}

bool RobotMoveit::moveToPose(RobotMoveit::WhichArm arm, std::string pose_name, bool waitForMotion)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	group->setNamedTarget(pose_name);
	group->setPlanningTime(10);

	ROS_INFO_STREAM("moveToPose: Moving arm to pose " << pose_name );

	bool success;

	if(waitForMotion)
	{
		success = group->move();
	}
	else
	{
		success = group->asyncMove();
		if(arm == RobotMoveit::LEFT)
			leftMotionInProgress = true;
		else
			rightMotionInProgress = true;
	}

	if(!success)
		ROS_ERROR("moveToPose: Failed to move to pose");

	return success;
}

bool RobotMoveit::getKinematicModelJointValues(std::vector<double> joint_values_left_arm, std::vector<double> joint_values_right_arm)
{
      // Get joint values
	  kinematic_state->copyJointGroupPositions("right_arm", joint_values_right_arm);
	  kinematic_state->copyJointGroupPositions("left_arm", joint_values_left_arm);
	  
	  /* Print joint names and values */
	  for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
	  {
	    ROS_INFO("Joint %s: %f", joint_names_right_arm[i].c_str(), joint_values_right_arm[i]);
	  }

	  for(std::size_t i = 0; i < joint_names_left_arm.size(); ++i)
	  {
	    ROS_INFO("Joint %s: %f", joint_names_left_arm[i].c_str(), joint_values_left_arm[i]);
	  }

	  return true;
}

bool RobotMoveit::setKinematicModelJointValues(std::vector<double> joint_values_left_arm, std::vector<double> joint_values_right_arm)
{

	// Get joint values
	kinematic_state->setJointGroupPositions("right_arm", joint_values_right_arm);
	kinematic_state->setJointGroupPositions("left_arm", joint_values_left_arm);

	// Check joint limits
	if(kinematic_state->satisfiesBounds())
	{
		ROS_INFO("Current kinematic model state is valid");
	}
	else
	{
		ROS_WARN("Current kinematic model state is not valid");
	}

	// Enforce joint limits
	kinematic_state->enforceBounds();
	// and check again
	if(kinematic_state->satisfiesBounds())
	{
		ROS_INFO("Current kinematic model state is valid");
	}
	else
	{
		ROS_WARN("Current kinematic model state is not valid");
		return false;
	}

	return true;
}

bool RobotMoveit::kinematicModelFK() {return true;}
bool RobotMoveit::kinematicModelIK() {return true;}
bool RobotMoveit::kinematicModelJacobian() {return true;}


bool RobotMoveit::executeJointGoal(RobotMoveit::WhichArm arm, std::vector<double> joint_values)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	group->setJointValueTarget(joint_values);
	bool success = group->plan(my_plan);

	ROS_INFO("Planning Joint goal %s",success?"SUCCEEDED":"FAILED");

	// TODO: Publish to RVIZ
//	  ROS_INFO("Visualizing plan 1 (again)");
//	  display_trajectory.trajectory_start = my_plan.start_state_;
//	  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//	  display_publisher.publish(display_trajectory);
//	  /* Sleep to give Rviz time to visualize the plan. */
//	  sleep(5.0);

	if(success)
	{
		group->move();
		return true;
	}

	return false;
}

bool RobotMoveit::executeCarteGoal(RobotMoveit::WhichArm arm, geometry_msgs::Pose target, bool waitForMotion)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	group->setPoseTarget(target);
	group->setStartStateToCurrentState();

	bool success = group->plan(my_plan);		// Note: this gets stuck if the ros::AsyncSpinner has too few threads

	ROS_INFO("Planning Cartesian goal %s",success?"SUCCEEDED":"FAILED");

	// TODO: Publish to RVIZ
//	  ROS_INFO("Visualizing plan 1 (again)");
//	  display_trajectory.trajectory_start = my_plan.start_state_;
//	  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//	  display_publisher.publish(display_trajectory);
//	  /* Sleep to give Rviz time to visualize the plan. */
//	  sleep(5.0);

	if(success)
	{
		/*
		if(waitForMotion)
		{
			group->move();
		}
		else
		{
			group->asyncMove();
			if(arm == RobotMoveit::LEFT)
				leftMotionInProgress = true;
			else
				rightMotionInProgress = true;
		}
		*/
		group->execute(my_plan);
		return true;
	}

	return false;
}

bool RobotMoveit::safeCarteGoal(RobotMoveit::WhichArm arm, geometry_msgs::Pose target, int nHold, double dx, double fractionThreshold)
{

	  std::vector<geometry_msgs::Pose> waypoints;
	  geometry_msgs::Pose start_pose;

	  // Get starting position
//	  getEndeffectorPose(BaxterMoveit::RIGHT,&start_pose);
//	  for(int i=0;i<nHold;i++)
//		  waypoints.push_back(start_pose);

//	  for(int i=0;i<nHold;i++)			// Hold final position for better accuracy
		  waypoints.push_back(target);

	  return executeCartePath(arm, waypoints, dx, fractionThreshold);

}

bool RobotMoveit::executeCarteGoalWithConstraint(RobotMoveit::WhichArm arm, geometry_msgs::Pose target)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	std::string arm_name;
	switch(arm)
	{
	case RobotMoveit::LEFT:
		arm_name = "left";
		break;
	case RobotMoveit::RIGHT:
		arm_name = "right";
		break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = arm_name+"_gripper";//"_lower_forearm";
	ocm.header.frame_id = "base";//"torso";
//	ocm.orientation.x = -0.6768 ;
//	ocm.orientation.y = -0.2047  ;
//	ocm.orientation.z = -0.2047;
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	group->setPathConstraints(test_constraints);


	// will only work if the current state already satisfies the path constraints
	robot_state::RobotState start_state(*group->getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
//	const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(group->getName());
//	start_state.setFromIK(joint_model_group, start_pose2);
	group->setStartState(start_state);

	group->setPoseTarget(target);

	bool success = group->plan(my_plan);

	ROS_INFO("Planning Cartesian goal %s",success?"SUCCEEDED":"FAILED");

	// TODO: Publish to RVIZ
//	  ROS_INFO("Visualizing plan 1 (again)");
//	  display_trajectory.trajectory_start = my_plan.start_state_;
//	  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//	  display_publisher.publish(display_trajectory);
//	  /* Sleep to give Rviz time to visualize the plan. */
//	  sleep(5.0);

	if(success)
	{
		group->move();
		group->clearPathConstraints();
		return true;
	}

	group->clearPathConstraints();
	return false;
}

bool RobotMoveit::executeJointPath(RobotMoveit::WhichArm arm, std::vector< std::vector<double> > joint_waypoints)
{

	return true;
}

bool RobotMoveit::executeCartePath(RobotMoveit::WhichArm arm, std::vector<geometry_msgs::Pose> waypoints, double dx, double fractionThreshold)
{
/*
 * http://answers.ros.org/question/74776/cartesian-controller-for-ros/
 * https://groups.google.com/forum/#!msg/moveit-users/x5FwalM5ruk/9OpXslS8x2YJ
 * https://groups.google.com/forum/#!msg/moveit-users/MOoFxy2exT4/JKFeSGASPMMJ
 *
 * call move_group->ComputeCartesianPath()executeCartePath with your sequence of cartesian points

    NOTE: this function accepts a std::vector<Pose>, so you'll need to convert your cartesian positions using tf::poseEigenToMsg()
    this returns a RobotTrajectory.

 * send your trajectory to MoveIt

    the easiest way I've found is to use the move_group node's ExecuteKnownTrajectory service, which is available on my setup at "/execute_kinematic_path".
    alternatively, you could probably send it to the TrajectoryExecutionManager object directly, as shown here, but that seems messy
 */


	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	if(dx > 0.15)
	{
		ROS_ERROR("BaxterMoveit::executeCartePath -> dt step size is too large.");
		return false;
	}


	moveit_msgs::RobotTrajectory trajectory;
	group->setPlanningTime(5.0);

	double fraction = group->computeCartesianPath(waypoints,
	                                              dx,  		    // eef_step (step size in meters)
	                                              0.0,   		// jump_threshold
	                                              trajectory,	// result
												  true);		// collision avoidance

	//std::cout<<trajectory<<"\n\n";

	if(fraction == -1)
	{
		ROS_ERROR("Computing the Cartesian path failed.");
		return false;
	}
	if(fraction < fractionThreshold)
	{
		ROS_ERROR("The computed Cartesian path achieved %.2f%% of the given path.",fraction * 100.0);
		return false;
	}
	if(0 <= fraction && fraction < 1)
	{
		ROS_WARN("The computed Cartesian path achieved %.2f%% of the given path.",fraction * 100.0);
	}

	std::string arm_group;
	switch(arm)
	{
	case RobotMoveit::LEFT:
		arm_group = "left_arm";
		break;
	case RobotMoveit::RIGHT:
		arm_group = "right_arm";
		break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}

	// Modify trajectory to include velocities
	// 1) create a RobotTrajectory object
	robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), arm_group);

	// 2) get a RobotTrajectory from trajectory
	rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
/*
	// 3) create a IterativeParabolicTimeParameterization object
	trajectory_processing::IterativeParabolicTimeParameterization iptp;

	// 4) compute computeTimeStamps
	bool success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

	// Get RobotTrajectory_msg from RobotTrajectory
	rt.getRobotTrajectoryMsg(trajectory);
*/
	// Plan and execute the trajectory
	my_plan.trajectory_ = trajectory;

	if(!group->execute(my_plan))
	{
		return false;
	}

	return true;
}


bool RobotMoveit::motionComplete()
{

	return true;
}

bool RobotMoveit::cancelMotion()
{

	return true;
}
