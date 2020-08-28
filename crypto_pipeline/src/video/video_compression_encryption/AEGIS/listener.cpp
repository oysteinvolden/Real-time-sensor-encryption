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
#include "aegis_128.h"
#include "encoder.h"

#define TAG_SIZE 16


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

  //ros::Rate loop_rate(10);

  // define key and IV once, key load only performed once for AEGIS
  // IV is only initialized at talker side

  u8 key[16] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
               	      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  aegis_state cs;
  aegis_load_key(&cs, (u32*)key);

  // initialize buffer for received iv - assume size is known
  u8 iv[IV_SIZE] = {0};
  u8 tag[TAG_SIZE] = {0};

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node **

    // start time - decryption
    start1 = std::chrono::system_clock::now();
    
    // copy incomming message 
    sensor_msgs::CompressedImage listener_msg_copy;
    listener_msg_copy = listener_msg; 


    // define data size
    int pt_length = listener_msg.data.size() - IV_SIZE - TAG_SIZE;
    
    if(pt_length > 0){

        // the front of the message received from talker is loaded to iv
      std::memcpy(iv, &listener_msg.data[0], IV_SIZE);

      // resize to original size without iv
      listener_msg_copy.data.resize(pt_length);

      // authenticted decryption    
      if (!aegis_decrypt_packet(&cs, &listener_msg_copy.data[0], &listener_msg.data[IV_SIZE], iv, (u32*)iv, (u32*)&listener_msg.data[IV_SIZE+pt_length], IV_SIZE, pt_length))
	    {
		    std::cout << "Invalid tag!\n";
		    exit(1);
	    }

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