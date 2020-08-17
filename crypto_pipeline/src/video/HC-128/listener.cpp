// ROS libraries
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

#define BLOCKSIZE 16

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_listener.txt";
std::ofstream log_time_delay(path_log);

// Create a container for the data received from talker
sensor_msgs::Image listener_msg;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){
  
  listener_msg = *msg;

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // subscribe for encrypted stream from talker
  ros::Subscriber encryptedImageSubscriber = n.subscribe("/encrypted_stream_from_talker", 1000, cameraCallback);

  // recovered image publisher
  ros::Publisher recoveredImagePublisher = n.advertise<sensor_msgs::Image>("/recovered_stream_listener", 1000);

  // define key only once
  std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
  u32 key[4];
  hex2stringString((u8*)key, hexkey.data(), 32);

  // initialize buffer to contain iv - assume size is known  
  u32 iv[4] = {0};

  // Create decryption object
  hc128_state d_cs;


  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

     // ** RECOVER **

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::Image listener_msg_copy;
    listener_msg_copy = listener_msg;

    // define data size
    int size = listener_msg.data.size() - sizeof(iv);
    
    if(size > 0){
 
      // the front of the message received from talker is loaded to iv
      std::memcpy(iv, &listener_msg.data[0], sizeof(iv));

      // resize to original size without iv
      listener_msg_copy.data.resize(size);

      // initialize cipher and decrypt
      hc128_initialize(&d_cs, key, iv);
      hc128_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[sizeof(iv)], size);

      // measure elapsed time - decryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;

      // publish recovered video stream
      recoveredImagePublisher.publish(listener_msg_copy);
    }

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
