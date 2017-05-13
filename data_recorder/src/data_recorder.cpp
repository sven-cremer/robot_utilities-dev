/*
 * data_recorder.cpp
 *
 *  Created on: May 9, 2017
 *      Author: Sven Cremer
 */

#include <data_recorder/data_recorder.h>

DataRecorder::DataRecorder(std::string topic_name)
{
	topic = topic_name;
}


DataRecorder::~DataRecorder()
{

}


void DataRecorder::start(std::string file_name)
{
	bag.open(file_name.c_str(), rosbag::bagmode::Write);
	//bag.write(topic.c_str());
}
