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


// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

#define BLOCKSIZE 16

// create a container for the data received from rosbag and listener
sensor_msgs::Image talker_msg;
sensor_msgs::Image talker_msg_from_list;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  talker_msg = *msg;

}

void cameraCallback2(const sensor_msgs::ImageConstPtr& msg){

  talker_msg_from_list = *msg;

}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_talker", 1000);

  // recovereed image publisher
  ros::Publisher recoveredImagePublisher = n.advertise<sensor_msgs::Image>("/recovered_stream_talker", 1000);

  // subscribe for rosbag image topic
  ros::Subscriber rosbagImageSubscriber = n.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback);

  // subscribe for encrypted image sent back  
  ros::Subscriber encryptedImageSubscriber = n.subscribe("/encrypted_stream_from_listener", 1000, cameraCallback2);

  
  

  while (ros::ok())
  {
    
    // start time
    start = std::chrono::system_clock::now();

    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node

    // define data size
    int size = talker_msg.data.size();

    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;


    // define key and IV
    std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
	  std::string hexIv = "288FF65DC42B92F960C72E95FC63CA31";    
    
    u32 key[4];
    hex2stringString((u8*)key, hexkey.data(), 32);
    u32 iv[4];
    hex2stringString((u8*)iv, hexIv.data(), 32);

    hc128_state e_cs;
	  hc128_initialize(&e_cs, key, iv);

    if(size > 0){
      hc128_process_packet(&e_cs, &talker_msg_copy.data[0], &talker_msg.data[0], size);
      encryptedImagePublisher.publish(talker_msg_copy);
    }  
    
    	  


    // ** PART3: listen for received ROS messages from listener node, then decrypt and publish recovered video **
    int size2 = talker_msg_from_list.data.size();

    if(size2 > 0){

      // RECOVER 
      sensor_msgs::Image talker_msg_from_list_copy;
      talker_msg_from_list_copy = talker_msg_from_list;
     
      // initialize cipher
      hc128_state d_cs;
	    hc128_initialize(&d_cs, key, iv);

      hc128_process_packet(&d_cs, &talker_msg_from_list_copy.data[0], &talker_msg_from_list.data[0], size2);
      
      // publish recovered video stream
      recoveredImagePublisher.publish(talker_msg_from_list_copy);


    }
      
    // increment key so its unique for each cryptographical operation
    hexkey += "1";

    // measure elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "RTT: " << elapsed_seconds.count() << std::endl;

    ros::spinOnce();
  }
  


  return 0;
}