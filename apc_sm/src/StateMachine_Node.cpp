#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apc_sm/pickUpMachine.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "StateMachine_Node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    std::cout << "\nBuilding a state machine";
	PickupMachine pm;
	pm.initiate();
	pm.process_event(FindObject());
	pm.process_event(pickObject());
	pm.process_event(findDestination());
	pm.process_event(placeObject());
	pm.process_event(goHome());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
