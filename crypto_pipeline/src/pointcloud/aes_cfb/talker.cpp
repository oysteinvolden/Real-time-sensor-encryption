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

// Create a container for the data received from rosbag 
sensor_msgs::PointCloud2 talker_msg; 


// callback for rosbag
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

  u8 key[BLOCKSIZE] = {0};
  u32 iv[BLOCKSIZE/4] = {0};
  cipher_state e_cs;


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
   
      // initialize cipher and encrypt
      cfb_initialize_cipher(&e_cs, key, iv);
      cfb_process_packet(&e_cs, &talker_msg.data[0], &talker_msg_copy.data[0], size_cloud, ENCRYPT);

      // measure elapsed time - encryption operation
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
