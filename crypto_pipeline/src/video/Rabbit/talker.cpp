// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>


// general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


//crypto
#include "rabbit.h"
#include "encoder.h"


// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_listener.txt";
std::ofstream log_time_delay(path_log);


// Create a container for the data received from talker
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

  
  while (ros::ok()){

    // ** PART 1:  listen for ROS messages from rosbag, then encrypt and send to talker node

    
    // ** ENCRYPTION **

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;

    int size = talker_msg.data.size();

    // key schedule is only performed once for each secret key
    std::string keyString = "00000000000000000000000000000000";
    u8 key[keyString.size()/2];
    hex2stringString(key, keyString.data(), keyString.size());

    rabbit_state cs;
    rabbit_key_setup(&cs, (u32*)key);
   
    // define IV  
    std::string ivString = "597E26C175F573C3";
    u8 iv[ivString.size()/2];
    hex2stringString(iv, ivString.data(), ivString.size());

    // Load key and encrypt 
    rabbit_iv_setup(&cs, (u32*)iv);
    rabbit_process_packet(&cs, &talker_msg_copy.data[0], &talker_msg.data[0], size);

    // measure elapsed time - decryption
    end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    if(size != 0){
      log_time_delay << elapsed_seconds1.count() << std::endl;
    }
 
    // publish encrypted video stream
    encryptedImagePublisher.publish(talker_msg_copy);  

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
