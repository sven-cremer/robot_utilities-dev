/*
 * data_recorder.h
 *
 *  Created on: May 9, 2017
 *      Author: Sven Cremer
 */

#ifndef DATA_RECORDER_DATA_RECORDER_H_
#define DATA_RECORDER_DATA_RECORDER_H_


#include <ros/ros.h>
//#include <ros/package.h>
//#include <rosbag/bag.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

class DataRecorder
{
public:

	DataRecorder(std::vector<std::string> param_topics,
                 std::vector<std::string> param_fnames,
				 std::string param_dataDir,
				 std::string param_path);
	~DataRecorder();

	void start(int file_number);
	void start();
	void stop();

private:

	//rosbag::Bag bag;

	std::vector<std::string> topics;
	std::vector<std::string> fnames;
	std::string dataDir;
	std::string path;

	void start(std::string pathDataDir);
	void systemStart(std::string topic, std::string fname, std::string pathDataDir);
	void systemStop (std::string topic);

	void createDataFolder(std::string pathDataDir);
};


#endif /* DATA_RECORDER_DATA_RECORDER_H_ */
