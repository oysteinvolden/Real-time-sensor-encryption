// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// point cloud
#include <sensor_msgs/PointCloud2.h>

// general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>
#include <thread>

//crypto
#include "sosemanuk.h"
#include "encoder.h"

#define BLOCKSIZE 16


// Create a container for the data received from talker
sensor_msgs::PointCloud2 listener_msg;


void lidarCallback2(const sensor_msgs::PointCloud2ConstPtr& msg){

  listener_msg = *msg;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // point cloud publisher - from listener
  ros::Publisher lidar_pub2 = n.advertise<sensor_msgs::PointCloud2>("/encrypted_points_from_listener", 1000);

  // point cloud publisher - from listener
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber encryptedPointCloud = n.subscribe("/encrypted_points_from_talker", 1000, lidarCallback2);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker

    // define data size
    u64 size_cloud = listener_msg.row_step * listener_msg.height;

    // RECOVER
    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    std::string keyString = "0DA416FE03E36529FB9BEA70872F0B5D";
    u8 key[keyString.size()/2];
    hex2stringString(key, keyString.data(), keyString.size());

	  std::string ivString = "D404755728FC17C659EC49D577A746E2";
    u8 iv[ivString.size()/2];
	  hex2stringString(iv, ivString.data(), ivString.size()); 

    // initialize cipher
    sosemanuk_state d_cs;
	  sosemanuk_load_key(&d_cs, key, keyString.size()/2);
	  sosemanuk_load_iv(&d_cs, (u32*)iv);


    if(size_cloud > 0){
      sosemanuk_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[0], size_cloud);
      lidar_pub.publish(listener_msg_copy);
    }
	  

    // ENCRYPT 
    sensor_msgs::PointCloud2 listener_msg_copy2;
    listener_msg_copy2 = listener_msg_copy;

    hex2stringString(key, keyString.data(), keyString.size());
    hex2stringString(iv, ivString.data(), ivString.size());

    // initialize cipher
    sosemanuk_state e_cs;

    // Load key and iv
	  sosemanuk_load_key(&e_cs, key, keyString.size()/2);
	  sosemanuk_load_iv(&e_cs, (u32*)iv);

    if(size_cloud > 0){
      sosemanuk_process_packet(&e_cs, &listener_msg_copy2.data[0], &listener_msg_copy.data[0], size_cloud);
      lidar_pub2.publish(listener_msg_copy2);
    }

    keyString += "1";

    ros::spinOnce();
    
  }

  return 0;
}