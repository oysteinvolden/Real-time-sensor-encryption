// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>


// general
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>

//crypto
#include "hc128.h"
#include "encoder.h"
#include "hmac.h"

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_listener.txt";
std::ofstream log_time_delay(path_log);

// We will use the standard 128-bit HMAC-tag.
#define TAGSIZE 16

#define AES_BLOCKSIZE 16

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

  u8 a_key[HMAC_KEYLENGTH] = {0};
  u8 e_key[AES_BLOCKSIZE] = {0};

  // Instantiate and initialize a HMAC struct
  hmac_state a_cs;
  hmac_load_key(&a_cs, a_key, HMAC_KEYLENGTH);

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
    int size = listener_msg.data.size() - TAGSIZE - HC128_IV_SIZE;

    
    if(size > 0){

      // Validate the tag over the IV and the ciphertext. If the(IV || Ciphertext, Tag)-pair is
	    // not valid, the ciphertext is NOT decrypted.
      if ( !(tag_validation(&a_cs, &listener_msg.data[HC128_IV_SIZE+size], &listener_msg.data[0], HC128_IV_SIZE+size, TAGSIZE)) ) {
	      std::cout << "Invalid tag!" << std::endl;
      }
     
      // resize to original size without tag and iv
      listener_msg_copy.data.resize(size);

      // Initialize cipher with new IV. The IV sits at the front of the msg.
      hc128_initialize(&d_cs, (u32*)e_key, (u32*)&listener_msg.data[0]);

      // Decrypt. The ciphertext sits after the IV 
      hc128_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[HC128_IV_SIZE], size);

      // measure elapsed time - decryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;
      
      // publish recovered image
      recoveredImagePublisher.publish(listener_msg_copy);      
    }
    

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
