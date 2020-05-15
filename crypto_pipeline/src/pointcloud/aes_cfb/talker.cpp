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
#include "aes_cfb.h"


#define BLOCKSIZE 16

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
   
    u8 key[BLOCKSIZE] = {0};
	  u32 iv[BLOCKSIZE/4] = {0};
    cipher_state e_cs;
	  cfb_initialize_cipher(&e_cs, key, iv);
    cfb_process_packet(&e_cs, &talker_msg.data[0], &talker_msg_copy.data[0], size_cloud, ENCRYPT);

    // measure elapsed time - encryption operation
    end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    if(size_cloud != 0){
      log_time_delay << elapsed_seconds1.count() << " ";
    }

    // publish encrypted point cloud
    lidar_pub.publish(talker_msg_copy);
      

    // ** PART3: listen for received ROS messages from listener node, then decrypt and publish recovered point cloud **

     // RECOVER 
    sensor_msgs::PointCloud2 talker_msg_from_list_copy;
    talker_msg_from_list_copy = talker_msg_from_list;

    // start time - decryption 
    start2 = std::chrono::system_clock::now();

    // define data size
    int size_cloud2 = talker_msg_from_list.data.size();
      
    cipher_state d_cs;
	  cfb_initialize_cipher(&d_cs, key, iv);
	  cfb_process_packet(&d_cs, &talker_msg_from_list.data[0], &talker_msg_from_list_copy.data[0], size_cloud2, DECRYPT);

    // measure elapsed time - decryption operation
    end2 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
    if(size_cloud2 != 0){
      log_time_delay << elapsed_seconds2.count() << std::endl;
    }

    // publish recovered data
    lidar_recovered.publish(talker_msg_from_list_copy);

    ros::spinOnce();
  }
  
  log_time_delay.close();


  return 0;
}