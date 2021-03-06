//
// Created by wei on 5/22/18.
//

#include "common/common_utils.h"
#include "common/ConfigParser.h"
#include "common/TimeLogger.h"
#include "core/SurfelWarpSerial.h"
#include <boost/filesystem.hpp>
#include <ros/console.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "SurfelWarp");
    ros::NodeHandle nh;


	//Get the config path
    std::string default_config_path = "/thesis_project/src/datasets_thesis/datasets/volumedeform/minion_config.json";
	std::string config_path;
    nh.param("dataset_config_file", config_path, default_config_path);


	//Parse it
    auto& config = surfelwarp::ConfigParser::Instance();
	config.ParseConfig(config_path);

	//The context
//	auto context = surfelwarp::initCudaContext();

	//Save offline
    bool offline_rendering = false;

	//The processing loop
    surfelwarp::SurfelWarpSerial fusion(nh);

	fusion.ProcessFirstFrame();
	for(auto i = 0; i < config.num_frames(); i++){
        ROS_INFO_STREAM("The " << i << "th Frame");
//        ROS_INFO_STREAM("hoi");
//		LOG(INFO) << "The " << i << "th Frame";
        surfelwarp::TimeLogger::printTimeLog("start to process a new frame");
		fusion.ProcessNextFrameWithReinit(offline_rendering);
	}
	
	//destroyCudaContext(context);
}
