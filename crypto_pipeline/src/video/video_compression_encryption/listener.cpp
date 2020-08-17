// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>

// opencv libraries
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>

// general
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

// log time delay decryption and decompression
const char *path_log1="time_delay_decryption_listener.txt";
std::ofstream log_time_delay1(path_log1);
const char *path_log2="time_delay_decompression_listener.txt";
std::ofstream log_time_delay2(path_log2);

// Create a container for the data received from talker
sensor_msgs::CompressedImage listener_msg;


void cameraCallback(const sensor_msgs::CompressedImageConstPtr& msg){
  
  listener_msg = *msg;
  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // subscribe for unencrypted stream from talker
  ros::Subscriber imageSubscriber = n.subscribe("/camera_array/cam0/image_raw/compressed", 1000, cameraCallback);

  // unencrypted image publisher
  ros::Publisher imagePublisher = n.advertise<sensor_msgs::Image>("/recovered_stream_from_listener", 1000);

  //ros::Rate loop_rate(100);

  u8 key[BLOCKSIZE] = {0};
  u32 iv[BLOCKSIZE/4] = {0};
  cipher_state d_cs;

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node **

    // start time - decryption
    start1 = std::chrono::system_clock::now();
    
    // copy incomming message 
    sensor_msgs::CompressedImage listener_msg_copy;
    listener_msg_copy = listener_msg; 


    // define data size
    int size = listener_msg.data.size();
    
    if(size > 0){

      // %%%% INITIALIZE CIPHER AND DECRYPT %%%%

	    cfb_initialize_cipher(&d_cs, key, iv);
	    cfb_process_packet(&d_cs, &listener_msg.data[0], &listener_msg_copy.data[0], size, DECRYPT);

      // measure elapsed time - decryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay1 << elapsed_seconds1.count() << std::endl;
      //std::cout << "decryption latency: " << elapsed_seconds1.count() << std::endl;
        

      // %%%% DECOMPRESSION %%%%

      // start time - decryption
      start2 = std::chrono::system_clock::now();

      cv::Mat decoded_image = cv::imdecode(cv::Mat(listener_msg_copy.data), 0); 

      // convert opencv image to ROS
      cv_bridge::CvImage out_msg;
      out_msg.header = listener_msg_copy.header; 
      out_msg.encoding = sensor_msgs::image_encodings::MONO8; 
      out_msg.image = decoded_image; 

      // measure elapsed time - decompression
      end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
      log_time_delay2 << elapsed_seconds2.count() << std::endl;
      //std::cout << "decompression latency: " << elapsed_seconds2.count() << std::endl;
      
      // publish decrypted and decompressed image
      imagePublisher.publish(out_msg.toImageMsg());   

    }
    
    //loop_rate.sleep();
    ros::spinOnce();
  }

  log_time_delay1.close();
  log_time_delay2.close();

  

  return 0;
}