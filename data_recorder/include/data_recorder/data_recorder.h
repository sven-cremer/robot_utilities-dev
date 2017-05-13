/*
 * data_recorder.h
 *
 *  Created on: May 9, 2017
 *      Author: sven
 */

#ifndef DATA_RECORDER_DATA_RECORDER_H_
#define DATA_RECORDER_DATA_RECORDER_H_


#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

class DataRecorder
{
public:

	DataRecorder(std::string topic_name);
	~DataRecorder();

	void start(std::string file_name);
	void stop();

private:

	rosbag::Bag bag;

	std::string topic;

};


#endif /* DATA_RECORDER_DATA_RECORDER_H_ */
