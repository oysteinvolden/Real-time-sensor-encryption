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
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber encryptedPointCloud = n.subscribe("/encrypted_points_from_talker", 1000, lidarCallback2);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker

    // ** RECOVER **

    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    // define data size
    int size_cloud = listener_msg.data.size();


    std::string keyString = "0DA416FE03E36529FB9BEA70872F0B5D";
    u8 key[keyString.size()/2];
    hex2stringString(key, keyString.data(), keyString.size());

	  std::string ivString = "D404755728FC17C659EC49D577A746E2";
    u8 iv[ivString.size()/2];
	  hex2stringString(iv, ivString.data(), ivString.size()); 

    if(size_cloud > 0){
      // initialize cipher
      sosemanuk_state d_cs;
	    sosemanuk_load_key(&d_cs, key, keyString.size()/2);
	    sosemanuk_load_iv(&d_cs, (u32*)iv);

      sosemanuk_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[0], size_cloud);

      // measure elapsed time - decryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << " ";
      

      lidar_pub.publish(listener_msg_copy);
    }
	  

    // ** ENCRYPT **

    sensor_msgs::PointCloud2 listener_msg_copy2;
    listener_msg_copy2 = listener_msg_copy;

    // start time - encryption
    start2 = std::chrono::system_clock::now();

    hex2stringString(key, keyString.data(), keyString.size());
    hex2stringString(iv, ivString.data(), ivString.size());

    if(size_cloud > 0){
      // initialize cipher
      sosemanuk_state e_cs;

      // Load key and iv
	    sosemanuk_load_key(&e_cs, key, keyString.size()/2);
	    sosemanuk_load_iv(&e_cs, (u32*)iv);

      sosemanuk_process_packet(&e_cs, &listener_msg_copy2.data[0], &listener_msg_copy.data[0], size_cloud);

      // measure elapsed time - encryption
      end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
      log_time_delay << elapsed_seconds2.count() << std::endl;
      
      lidar_pub2.publish(listener_msg_copy2);
    }

    keyString += "1";

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}