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

  // encrypted image publisher
  //ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_listener", 1000);

  // recovered image publisher
  ros::Publisher recoveredImagePublisher = n.advertise<sensor_msgs::Image>("/recovered_stream_listener", 1000);


  while (ros::ok()){

      // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

      // start time - decryption
      start1 = std::chrono::system_clock::now();

      int size = listener_msg.data.size();

      if(size > 0){

        // ** RECOVER **

        sensor_msgs::Image listener_msg_copy;
        listener_msg_copy = listener_msg;

        u8 key[BLOCKSIZE] = {0};
        u32 iv[BLOCKSIZE/4] = {0};

        // initialize cipher
	cipher_state d_cs;
	cfb_initialize_cipher(&d_cs, key, iv);

	cfb_process_packet(&d_cs, &listener_msg.data[0], &listener_msg_copy.data[0], size, DECRYPT);

        // measure elapsed time - decryption
        end1 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
        log_time_delay << elapsed_seconds1.count() << std::endl;
        

        // publish recovered video stream
        recoveredImagePublisher.publish(listener_msg_copy); 


        // ** ENCRYPT ** 
	/*
	// start time - encryption
        start2 = std::chrono::system_clock::now();

        sensor_msgs::Image listener_msg_copy2;
        listener_msg_copy2 = listener_msg_copy;

        // initialize cipher
        cipher_state e_cs;
	cfb_initialize_cipher(&e_cs, key, iv);

        cfb_process_packet(&e_cs, &listener_msg_copy.data[0], &listener_msg_copy2.data[0], size, ENCRYPT);

        // measure elapsed time - encryption
        end2 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
        log_time_delay << elapsed_seconds2.count() << std::endl;
	
        // publish encrypted image via ROS
        encryptedImagePublisher.publish(listener_msg_copy2);
 	*/
    }
    
    ros::spinOnce();
  }

  log_time_delay.close();

  return 0;
}
