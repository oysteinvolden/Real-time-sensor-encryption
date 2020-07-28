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

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);


#define BLOCKSIZE 16


// create a container for the data received from rosbag and listener
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


  u8 key[BLOCKSIZE] = {0};
  u32 iv[BLOCKSIZE/4] = {0};
  cipher_state e_cs;

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

      // initialize cipher and encrypt
      cfb_initialize_cipher(&e_cs, key, iv);
      cfb_process_packet(&e_cs, &talker_msg.data[0], &talker_msg_copy.data[0], size, ENCRYPT);

      // measure elapsed time - encryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;
      
      // publish encrypted video stream
      encryptedImagePublisher.publish(talker_msg_copy);

    }

    ros::spinOnce();

  }
  
  log_time_delay.close();

  return 0;
}
