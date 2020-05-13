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
#include "aes_cfb.h"


#define BLOCKSIZE 16


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

  u8 key[BLOCKSIZE] = {0};
  u32 iv[BLOCKSIZE/4] = {0};

  ros::Rate loop_rate(50);

  while (ros::ok()){

      // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

      int size = listener_msg.data.size();

      if(size > 0){

        // ** RECOVER **

        sensor_msgs::Image listener_msg_copy;
        listener_msg_copy = listener_msg;

        // initialize cipher
	      cipher_state d_cs;
	      cfb_initialize_cipher(&d_cs, key, iv);


	      cfb_process_packet(&d_cs, &listener_msg.data[0], &listener_msg_copy.data[0], size, DECRYPT);

        // publish recovered video stream
        recoveredImagePublisher.publish(listener_msg_copy); 


        // ** ENCRYPT ** 

        sensor_msgs::Image listener_msg_copy2;
        listener_msg_copy2 = listener_msg_copy;

        // initialize cipher
        cipher_state e_cs;
	      cfb_initialize_cipher(&e_cs, key, iv);

        cfb_process_packet(&e_cs, &listener_msg_copy.data[0], &listener_msg_copy2.data[0], size, ENCRYPT);
	
        // publish encrypted image via ROS
        encryptedImagePublisher.publish(listener_msg_copy2);
 
    }
 
    loop_rate.sleep();
    ros::spinOnce();
  }

 

  return 0;
}