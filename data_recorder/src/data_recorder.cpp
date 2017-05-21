/*
 * data_recorder.cpp
 *
 *  Created on: May 9, 2017
 *      Author: Sven Cremer
 */

#include <data_recorder/data_recorder.h>

DataRecorder::DataRecorder(std::vector<std::string> param_topics,
                           std::vector<std::string> param_fnames,
                           std::string param_dataDir,
                           std::string param_path)
{
	topics      = param_topics;
	fnames      = param_fnames;
	dataDir     = param_dataDir;
	path        = param_path;

	pathDataDir = path + "/" + dataDir;

	if( topics.size() != fnames.size() )
	{
		std::cerr << "Number of topic and file names do not agree!\n";
	}
	if( !boost::filesystem::exists(path) )
	{
		std::cerr << "Path does not exist!\n";
	}
}


DataRecorder::~DataRecorder()
{
}

void DataRecorder::start(int file_number)
{
	std::ostringstream ss;
	ss << std::setw(2) << std::setfill('0') << file_number;

	pathDataDir = path + "/" + dataDir + "_" + ss.str();
	start();
}
void DataRecorder::start()
{
	createDataFolder();

	for(int i=0; i<topics.size();i++)
	{
		systemStart(topics[i], fnames[i]);
	}
	//bag.open(file_name.c_str(), rosbag::bagmode::Write);
	//bag.write(topic.c_str());
}

void DataRecorder::stop()
{
	for(int i=0; i<topics.size();i++)
	{
		systemStop(topics[i]);
	}
}

void DataRecorder::systemStart(std::string topic, std::string fileName, DataRecorder::DataType type)
{
	std::string cmd;

	if(type == DataRecorder::ROSBAG)
		cmd = "rosbag record " + topic  + " -O " + pathDataDir + "/" + fileName + ".bag &";		// Note: only works if recording entire topic, i.e this doesn't work: topic/subtopic
	else
		cmd = "rostopic echo -p " + topic  + " > " + pathDataDir + "/" + fileName + ".csv &";

	std::cout<<"$ "<<cmd.c_str()<<"\n";
	system( cmd.c_str() );
}

void DataRecorder::systemStop(std::string topic)
{
	  std::string cmd = "pkill -9 -f " + topic;
	  std::cout<<"$ "<<cmd.c_str()<<"\n";
	  system( cmd.c_str() );
}

void DataRecorder::createDataFolder()
{
	// Create experiment data directory
	boost::filesystem::path dir(pathDataDir);
	if(!boost::filesystem::create_directory(dir))
	{
		if( boost::filesystem::exists(dir) )
		{
			std::cout << "Warning: Experiment folder already exists ... will replace files!" << "\n";
		}
		else
		{
			std::cerr << "Failed to create experiment directory!" << "\n";
			//return 1;
		}
	}
}

std::string DataRecorder::getPathDataDir()
{
	return pathDataDir;
}

