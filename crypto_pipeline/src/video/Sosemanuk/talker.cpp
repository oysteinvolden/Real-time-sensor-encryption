// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>

//general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


//crypto
#include "sosemanuk.h"
#include "encoder.h"


// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);

// create a container for the data received from rosbag 
sensor_msgs::Image talker_msg;

// callback for rosbag
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


  // define key and IV once, key load only performed once for sosemanuk
  // IV is only defined at talker side
  std::string keyString = "0DA416FE03E36529FB9BEA70872F0B5D";
  u8 key[keyString.size()/2];
  hex2stringString(key, keyString.data(), keyString.size());

  sosemanuk_state e_cs;
  sosemanuk_load_key(&e_cs, key, keyString.size()/2);
  
  std::string ivString = "D404755728FC17C659EC49D577A746E2";
  u8 iv[ivString.size()/2];
  hex2stringString(iv, ivString.data(), ivString.size());

  while (ros::ok())
  {

    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node
   
    // ** ENCRYPTION **

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;
  
    // define data size
    int size = talker_msg.data.size(); 

    if(size > 0){

      // resize to include iv   
      talker_msg_copy.data.resize(size + ivString.size()/2);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_copy.data[0], iv, ivString.size()/2);

      // Load iv and encrypt
      sosemanuk_load_iv(&e_cs, (u32*)iv);  
      sosemanuk_process_packet(&e_cs, &talker_msg_copy.data[ivString.size()/2], &talker_msg.data[0], size);

      // increment for unique iv
      iv[3]++;

      // measure elapsed time - encryption 
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;

      // publish encrypted image
      encryptedImagePublisher.publish(talker_msg_copy);
    }
    
    ros::spinOnce();
  }
  
  log_time_delay.close();

  return 0;
}
