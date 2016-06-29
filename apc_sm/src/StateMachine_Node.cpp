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
		p.process_event(activate());
		p.process_event(navigate());
		/*// go to Open, call on_exit on Empty, then action, then on_entry on Open
		p.process_event(open_close());
		pstate(p);
		p.process_event(open_close());
		pstate(p);
		// will be rejected, wrong disk type
		p.process_event(object_detected("louie, louie", DISK_DVD));
		pstate(p);
		p.process_event(object_detected("louie, louie", DISK_CD));
		pstate(p);
		p.process_event(play());

		// at this point, Play is active
		p.process_event(sm::pause());
		pstate(p);
		// go back to Playing
		p.process_event(end_pause());
		pstate(p);
		p.process_event(sm::pause());
		pstate(p);
		p.process_event(stop());
		pstate(p);*/
		// event leading to the same state
		// no action method called as it is not present in the transition table
		//p.process_event(stop());
		//pstate(p);
		std::cout << "stop fsm" << std::endl;
		p.stop();

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
