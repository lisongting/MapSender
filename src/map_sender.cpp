#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <map_sender/MapStr.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "../include/map_sender/map_sender.h"
using namespace cv;
using namespace std;
namespace map_sender
{
MapSender::MapSender(){
	currentX = 0;
	currentY = 0;
	mapWidth = 0;
	mapHeight = 0;
	odomCount = 0;
}

MapSender::~MapSender(){}

bool MapSender::init(){
	ros::NodeHandle node("~");

	map_data_subscriber = node.subscribe("/map",1,&MapSender::mapDataReceived,this);
	location_subscriber = node.subscribe("/odom",10,&MapSender::odomReceived,this);
	map_string_publisher =node.advertise<map_sender::MapStr>("/map_string",1);

	//publish a init string to let another subscribers can subsribe successfully before the real data comes
	map_sender::MapStr first_str;
	first_str.base64 = " ";
	map_string_publisher.publish(first_str);
	
	return true;
}	

void MapSender ::start(){
	ros::Rate loop_rate(1);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void MapSender::mapDataReceived( nav_msgs::OccupancyGrid& map){
	mapWidth = map.info.width;
	mapHeight = map.info.height;
	// rawMapData = (int *)map.data;
	mapDataLength = map.data.size();

	//rawMapData = map.data;
	memcpy(&rawMapData,&map.data,sizeof(map.data));

}

void MapSender::odomReceived(const nav_msgs::Odometry& odom){
	odomCount++;
	if(odomCount<COUNTNUM){
		currentX = odom.pose.pose.position.x;
		currentY = odom.pose.pose.position.y;
		return ;
	}else if(mapWidth>0 && mapHeight>0 && mapDataLength>1000){
		//current location index in matrix area
		int currentXIndex = (int)(mapWidth * (25.0 + currentX) / 50.0);
		int currentYIndex = (int)(mapHeight * (25.0+currentY) / 50.0);
		mat = cv::Mat(mapWidth,mapHeight,CV_8UC3);
		int x,y;
		for(int i=0;i<mapDataLength;i++){
			x = i / mapWidth;
			y = i % mapWidth;
			if(rawMapData[i] == -1 ){
				//unknown area --> dark gray
				mat.at<Vec3b>(x,y)[0] = 100;
				mat.at<Vec3b>(x,y)[1] = 100;
				mat.at<Vec3b>(x,y)[2] = 100;
			}else if(rawMapData[i] == 0){
				//accessable area --> green
				mat.at<Vec3b>(x,y)[0] = 121;
				mat.at<Vec3b>(x,y)[1] = 213;
				mat.at<Vec3b>(x,y)[2] = 117;
			}else if(rawMapData[i] == 100){
				//obstacle   -->  white
				mat.at<Vec3b>(x,y)[0] = 240;
				mat.at<Vec3b>(x,y)[1] = 240;
				mat.at<Vec3b>(x,y)[2] = 240;
			}
			//xbot current area  --> dark red
			if(abs(x-currentXIndex<15)&&abs(y-currentYIndex<15)){
				mat.at<Vec3b>(x,y)[0] = 0;
				mat.at<Vec3b>(x,y)[1] = 0;
				mat.at<Vec3b>(x,y)[2] = 204;
			}
		}

		vector<uchar> vecImg;        //transform  Mat to  vector<uchar>  
	    	vector<int> params;  
	             params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	    	params.push_back(90);  
	    	//encode mat matrix into vector<uchar> 
	    	imencode(".png", mat, vecImg, params);  
 		
    		string base64_str = encode(vecImg.data(), vecImg.size()); 
		
		map_sender::MapStr map;
		map.base64 = base64_str;
		map_string_publisher.publish(map);

		cout<<"base64 String: "<<base64_str<<endl;
		odomCount = 0;
	}

}

//encode to base64 string
std::string MapSender::encode(const unsigned char* Data,int size){
    //encode table
    const char EncodeTable[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";  
    //the base64 string
    std::string strEncode;  
    unsigned char Tmp[4]={0};  
    int LineLength=0;  
    for(int i=0;i<(int)(size / 3);i++)  
    {  
        Tmp[1] = *Data++;  
        Tmp[2] = *Data++;  
        Tmp[3] = *Data++;  
        strEncode+= EncodeTable[Tmp[1] >> 2];  
        strEncode+= EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];  
        strEncode+= EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];  
        strEncode+= EncodeTable[Tmp[3] & 0x3F];  
        if(LineLength+=4,LineLength==76) {strEncode+="\r\n";LineLength=0;}  
    }  
    
    int Mod=size % 3;  
    if(Mod==1)  
    {  
        Tmp[1] = *Data++;  
        strEncode+= EncodeTable[(Tmp[1] & 0xFC) >> 2];  
        strEncode+= EncodeTable[((Tmp[1] & 0x03) << 4)];  
        strEncode+= "==";  
    }  
    else if(Mod==2)  
    {  
        Tmp[1] = *Data++;  
        Tmp[2] = *Data++;  
        strEncode+= EncodeTable[(Tmp[1] & 0xFC) >> 2];  
        strEncode+= EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];  
        strEncode+= EncodeTable[((Tmp[2] & 0x0F) << 2)];  
        strEncode+= "=";  
    }  
      
    return strEncode;  
}

}