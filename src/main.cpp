#include "../include/map_sender/map_sender.h"

using map_sender::MapSender;
int main(int argc,char **argv){
	MapSender mapSender;
	ros::init(argc,argv,"map_sender");

	if(mapSender.init()){
		ROS_INFO("MapSender is running ...");
		mapSender.start();
	}else{
		ROS_ERROR_STREAM("Init Error");
	}

	return 0;
}