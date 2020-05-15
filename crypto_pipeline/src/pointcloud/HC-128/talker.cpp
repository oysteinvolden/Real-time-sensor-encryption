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
sensor_msgs::PointCloud2 talker_msg_from_list;


// callback for rosbag
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg = *msg;

}

// callback for listener node
void lidarCallback2(const sensor_msgs::PointCloud2ConstPtr& msg){

  //std::cout << "test2" << std::endl;

  talker_msg_from_list = *msg;

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;


  // point cloud publisher - from talker
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/encrypted_points_from_talker", 1000);

  ros::Publisher lidar_recovered = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_talker", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/os1_cloud_node/points", 1000, lidarCallback); 

  // point cloud subscriber - from listener
  ros::Subscriber sub_list = n.subscribe<sensor_msgs::PointCloud2> ("/encrypted_points_from_listener", 1000, lidarCallback2); 


  while (ros::ok())
  {
    

    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node

    

   
    // ** ENCRYPTION **
    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    // define data size
    int size_cloud = talker_msg.data.size(); 

    std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
	  std::string hexIv = "288FF65DC42B92F960C72E95FC63CA31";    


    u32 key[4];
    hex2stringString((u8*)key, hexkey.data(), 32);

    u32 iv[4];
    hex2stringString((u8*)iv, hexIv.data(), 32);
    if(size_cloud > 0){
      hc128_state e_cs;
	    hc128_initialize(&e_cs, key, iv);
    
      hc128_process_packet(&e_cs, &talker_msg_copy.data[0], &talker_msg.data[0], size_cloud);

       // measure elapsed time - encryption operation
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << " ";
      
      lidar_pub.publish(talker_msg_copy);
    }
	  


    // ** PART3: listen for received ROS messages from listener node, then decrypt and publish recovered point cloud **

    
      
    // RECOVER 
    sensor_msgs::PointCloud2 talker_msg_from_list_copy;
    talker_msg_from_list_copy = talker_msg_from_list;

    // start time - decryption 
    start2 = std::chrono::system_clock::now();

    // define data size
    int size_cloud2 = talker_msg_from_list.data.size();

    if(size_cloud2 > 0){
      
      hc128_state d_cs;
	    hc128_initialize(&d_cs, key, iv);
    
      hc128_process_packet(&d_cs, &talker_msg_from_list_copy.data[0], &talker_msg_from_list.data[0], size_cloud2);

      hexkey += "1";

      // measure elapsed time - decryption operation
      end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
      log_time_delay << elapsed_seconds2.count() << std::endl;
      
      lidar_recovered.publish(talker_msg_from_list_copy);
    }
    

    ros::spinOnce();
  }

  log_time_delay.close();  


  return 0;
}