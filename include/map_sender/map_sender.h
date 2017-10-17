#ifndef MAP_SENDER_H
#define MAP_SENDER_H

#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <stdlib.h>
#include <map_sender/MapStr.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
//the frequency "odom" is 50hz
//but the frequency of "map"  is less active
#define COUNTNUM 35

namespace map_sender{

//MapSender is used to send map data in format  of base64-string 
class MapSender{

public:
	MapSender();
	~MapSender();
	bool init();
	void start();

private:
	int *rawMapData;
	cv::Mat mat;
	float currentX;
	float currentY;
	int odomCount;
	int mapWidth;
	int mapHeight;
	int mapDataLength;
	ros::Subscriber map_data_subscriber;
	ros::Subscriber location_subscriber;
	ros::Publisher map_string_publisher;

	std::string encode(const unsigned char* Data,int size);

	void mapDataReceived(const nav_msgs::OccupancyGrid& map);

	void odomReceived(const nav_msgs::Odometry& odom);

};
}


#endif
