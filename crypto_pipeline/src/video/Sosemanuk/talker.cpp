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
#include "sosemanuk.h"
#include "encoder.h"


// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

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

    // ** ENCRYPTION **
    
    std::string keyString = "0DA416FE03E36529FB9BEA70872F0B5D";
    u8 key[keyString.size()/2];
    hex2stringString(key, keyString.data(), keyString.size());

	  std::string ivString = "D404755728FC17C659EC49D577A746E2";
    u8 iv[ivString.size()/2];
	  hex2stringString(iv, ivString.data(), ivString.size());    

    // initialize cipher
    sosemanuk_state e_cs;

    // Load key and iv
	  sosemanuk_load_key(&e_cs, key, keyString.size()/2);
	  sosemanuk_load_iv(&e_cs, (u32*)iv);  

    if(size > 0){
      sosemanuk_process_packet(&e_cs, &talker_msg_copy.data[0], &talker_msg.data[0], size);
      encryptedImagePublisher.publish(talker_msg_copy);
    }


    // ** PART3: listen for received ROS messages from listener node, then decrypt and show recovered video **

    int size2 = talker_msg_from_list.data.size();

    if(size2 > 0){

      // RECOVER 
      sensor_msgs::Image talker_msg_from_list_copy;
      talker_msg_from_list_copy = talker_msg_from_list;

      // initialize cipher
      sosemanuk_state d_cs;
	    sosemanuk_load_key(&d_cs, key, keyString.size()/2);
	    sosemanuk_load_iv(&d_cs, (u32*)iv);
 
      sosemanuk_process_packet(&d_cs, &talker_msg_from_list_copy.data[0], &talker_msg_from_list.data[0], size2);      

      // publish recovered video stream
      recoveredImagePublisher.publish(talker_msg_from_list_copy);

    }

    // measure elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "RTT: " << elapsed_seconds.count() << std::endl;

    keyString += "1";

    ros::spinOnce();
  }
  


  return 0;
}