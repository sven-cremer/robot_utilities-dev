#include "ros/ros.h"
#include "std_msgs/String.h"
#include "apc_sm/pickUpMachine.h"
#include <sstream>
using namespace sm;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, "StateMachine_Node");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		std::cout << "\nBuilding a state machine";
		pickupMachine p;
		// needed to start the highest-level SM. This will call on_entry and mark the start of the SM
		p.start();
		pstate(p);
		p.process_event(activate());
		pstate(p);
		/*p.process_event(navigate());
		pstate(p);
		p.process_event(pick("water", BOTTLE));
		p.process_event(pick("drink", COKE));

		std::cout << "stop fsm" << std::endl;*/
		p.stop();

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
