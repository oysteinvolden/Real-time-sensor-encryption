// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//#include <cv_bridge/cv_bridge.h>


// general
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

  // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_listener", 1000);

  // recovered image publisher
  ros::Publisher recoveredImagePublisher = n.advertise<sensor_msgs::Image>("/recovered_stream_listener", 1000);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

    int size = listener_msg.data.size();
    
    if(size > 0){

      // ** RECOVER **

      sensor_msgs::Image listener_msg_copy;
      listener_msg_copy = listener_msg;

      std::string keyString = "0DA416FE03E36529FB9BEA70872F0B5D";
      u8 key[keyString.size()/2];
      hex2stringString(key, keyString.data(), keyString.size());

	    std::string ivString = "D404755728FC17C659EC49D577A746E2";
      u8 iv[ivString.size()/2];
	    hex2stringString(iv, ivString.data(), ivString.size()); 

      // initialize cipher
      sosemanuk_state d_cs;
	    sosemanuk_load_key(&d_cs, key, keyString.size()/2);
	    sosemanuk_load_iv(&d_cs, (u32*)iv);

      sosemanuk_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[0], size);
 
      // publish recovered video stream
      recoveredImagePublisher.publish(listener_msg_copy);      


      // ** ENCRYPT ** 

      sensor_msgs::Image listener_msg_copy2;
      listener_msg_copy2 = listener_msg_copy;

      hex2stringString(key, keyString.data(), keyString.size());
      hex2stringString(iv, ivString.data(), ivString.size());

      // initialize cipher
      sosemanuk_state e_cs;

      // Load key and iv
	    sosemanuk_load_key(&e_cs, key, keyString.size()/2);
	    sosemanuk_load_iv(&e_cs, (u32*)iv);

      sosemanuk_process_packet(&e_cs, &listener_msg_copy2.data[0], &listener_msg_copy.data[0], size);

      // publish encrypted video stream
      encryptedImagePublisher.publish(listener_msg_copy2);

      keyString += "1";

    }

    

    ros::spinOnce();
    
  }

  return 0;
}