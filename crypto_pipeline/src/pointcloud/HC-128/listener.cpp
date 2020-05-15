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

//crypto
#include "hc128.h"
#include "encoder.h"

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_listener.txt";
std::ofstream log_time_delay(path_log);

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
  ros::Publisher lidar_pub3 = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber encryptedPointCloud = n.subscribe("/encrypted_points_from_talker", 1000, lidarCallback2);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker

    // RECOVER
    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    // define data size
    int size_cloud = listener_msg.data.size();

    std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
	  std::string hexIv = "288FF65DC42B92F960C72E95FC63CA31";

    u32 key[4];
	  hex2stringString((u8*)key, hexkey.data(), 32);

	  u32 iv[4];
	  hex2stringString((u8*)iv, hexIv.data(), 32);

    if(size_cloud > 0){

      hc128_state d_cs;
	    hc128_initialize(&d_cs, key, iv);
    
      hc128_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[0], size_cloud);

      // measure elapsed time - decryption operation
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << " ";

      lidar_pub3.publish(listener_msg_copy);
    }
	  

    // ENCRYPT 
    sensor_msgs::PointCloud2 listener_msg_copy2;
    listener_msg_copy2 = listener_msg_copy;

    // start time - encryption
    start2 = std::chrono::system_clock::now();    

    //u32 key[4];
	  hex2stringString((u8*)key, hexkey.data(), 32);

	  //u32 iv[4];
	  hex2stringString((u8*)iv, hexIv.data(), 32);

    if(size_cloud > 0){

      hc128_state e_cs;
	    hc128_initialize(&e_cs, key, iv);
    
      hc128_process_packet(&e_cs, &listener_msg_copy2.data[0], &listener_msg_copy.data[0], size_cloud);

      // measure elapsed time - encryption operation
      end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
      log_time_delay << elapsed_seconds2.count() << std::endl;
      
      lidar_pub2.publish(listener_msg_copy2);
    }

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}