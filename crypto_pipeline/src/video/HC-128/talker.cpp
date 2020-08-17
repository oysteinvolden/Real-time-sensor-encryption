//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>

//general
#include <iostream>
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

#define BLOCKSIZE 16

// create a container for the data received from rosbag and listener
sensor_msgs::Image talker_msg;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  talker_msg = *msg;

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_talker", 1000);


  // subscribe for rosbag image topic
  ros::Subscriber rosbagImageSubscriber = n.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback);


  // define key and IV once
  // IV is only defined at talker side
  std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
  u32 key[4]; 
  hex2stringString((u8*)key, hexkey.data(), 32); 

  std::string hexIv = "288FF65DC42B92F960C72E95FC63CA31";    
  u32 iv[4]; 
  hex2stringString((u8*)iv, hexIv.data(), 32); 

  // create encryption object
  hc128_state e_cs;


  while (ros::ok())
  {
    
    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;

    // define data size
    int size = talker_msg.data.size();

    if(size > 0){

      // resize to include iv 
      talker_msg_copy.data.resize(size + hexIv.size()/2);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_copy.data[0], iv, hexIv.size()/2);

      // initialize and encrypt
      hc128_initialize(&e_cs, key, iv);
      hc128_process_packet(&e_cs, &talker_msg_copy.data[hexIv.size()/2], &talker_msg.data[0], size);

      // increment for unique iv
      iv[3]++;

      // measure elapsed time - encryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;
      
      // publish encrypted video stream
      encryptedImagePublisher.publish(talker_msg_copy);
    }  
    
    ros::spinOnce();

  }

  log_time_delay.close();
  


  return 0;
}
