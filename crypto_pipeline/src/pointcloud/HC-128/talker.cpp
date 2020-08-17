//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

//point cloud
#include <sensor_msgs/PointCloud2.h>

//general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>

//crypto
#include "hc128.h"
#include "encoder.h"

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);

// Create a container for the data received from rosbag and listener
sensor_msgs::PointCloud2 talker_msg; 


void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg = *msg;

}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;


  // point cloud publisher - from talker
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/encrypted_points_from_talker", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/os1_cloud_node/points", 1000, lidarCallback); 

  // define key and IV once, key load only performed once for HC-128
  // IV is only defined at talker side
  std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
  u32 key[4]; 
  hex2stringString((u8*)key, hexkey.data(), 32); 

  std::string hexIv = "288FF65DC42B92F960C72E95FC63CA31";    
  u32 iv[4]; 
  hex2stringString((u8*)iv, hexIv.data(), 32); 

  hc128_state e_cs;

  while (ros::ok())
  {
    
    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node
 
    // ** ENCRYPTION **
     
    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;

    // define data size
    int size_cloud = talker_msg.data.size(); 

    if(size_cloud > 0){

      // resize to include iv 
      talker_msg_copy.data.resize(size_cloud + hexIv.size()/2);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_copy.data[0], iv, hexIv.size()/2);
      
      // initialize and encrypt
      hc128_initialize(&e_cs, key, iv);
      hc128_process_packet(&e_cs, &talker_msg_copy.data[hexIv.size()/2], &talker_msg.data[0], size_cloud);

      // increment for unique iv
      iv[3]++;

      // measure elapsed time - encryption 
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;
      
      // publish encrypted point cloud
      lidar_pub.publish(talker_msg_copy);
    }
	  
    ros::spinOnce();

  }

  log_time_delay.close();  


  return 0;
}
