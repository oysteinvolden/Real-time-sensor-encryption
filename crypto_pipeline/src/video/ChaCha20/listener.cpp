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
#include "chacha.h"
#include "encoder.h"


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

  
  
  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

    
    // ** RECOVER **

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::Image listener_msg_copy;
    listener_msg_copy = listener_msg;

    int size = listener_msg.data.size();

     // Key and nonce in byte array order.
	  std::string key_string = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
	  std::string nonce_string = "000000000000004A00000000";

	  u8 key[32];
	  u8 nonce[12];

	  // Convert hex key and nonce to u8
	  hex2stringString(key, key_string.data(), key_string.size());
	  hex2stringString(nonce, nonce_string.data(), nonce_string.size());

    // Initialize the cipher to decrypt
	  chacha_state d_cs;
	  chacha20_initialize(&d_cs, (u32*)key, (u32*)nonce);

    // decrypt
    chacha20_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[0], size);

    // measure elapsed time - decryption
    end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    if(size != 0){
      log_time_delay << elapsed_seconds1.count() << std::endl;
    }
 
    // publish recovered video stream
    recoveredImagePublisher.publish(listener_msg_copy);      

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
