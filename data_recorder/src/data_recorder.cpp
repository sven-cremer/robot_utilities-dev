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

	std::string pathDataDir = path + "/" + dataDir + "_" + ss.str();
	start(pathDataDir);
}
void DataRecorder::start()
{
	std::string pathDataDir = path + "/" + dataDir;
	start(pathDataDir);
}
void DataRecorder::start(std::string pathDataDir)
{
	createDataFolder(pathDataDir);

	for(int i=0; i<topics.size();i++)
	{
		systemStart(topics[i], fnames[i], pathDataDir);
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

void DataRecorder::systemStart(std::string topic, std::string fileName, std::string pathDataDir)
{
	  std::string cmd = "rostopic echo -p " + topic  + " > " + pathDataDir + "/" + fileName + ".csv &";
	  std::cout<<"$ "<<cmd.c_str()<<"\n";
	  system( cmd.c_str() );
}

void DataRecorder::systemStop(std::string topic)
{
	  std::string cmd = "pkill -9 -f " + topic;
	  std::cout<<"$ "<<cmd.c_str()<<"\n";
	  system( cmd.c_str() );
}

void DataRecorder::createDataFolder(std::string pathDataDir)
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

