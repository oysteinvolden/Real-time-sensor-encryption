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

// log time delay compression and encryption
const char *path_log1="time_delay_compression_talker.txt";
std::ofstream log_time_delay1(path_log1);
const char *path_log2="time_delay_encryption_talker.txt";
std::ofstream log_time_delay2(path_log2);

// compression parameters
std::vector<int> compression_params;

// create containers for the compressed data 
sensor_msgs::CompressedImage talker_msg_compressed;



void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  // %%%% COMPRESSION %%%%

  // define start time 
  start1 = std::chrono::system_clock::now();

  // define header and format
  talker_msg_compressed.header = msg->header;
  //talker_msg_compressed.format = "png";
  talker_msg_compressed.format = "jpeg";

  // bridge ros message to opencv
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // PNG
  //compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); 
  //compression_params.push_back(3); // level 0 - 9 - tradeoff between file size and compression latency (the higher the latency, the lower the size)
  // JPEG
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); 
  compression_params.push_back(95); // 0-100

  // store compressed opencv image to outbut buffer (ros topic)
  //cv::imencode(".png", cv_ptr->image, talker_msg_compressed.data, compression_params);
  cv::imencode(".jpeg", cv_ptr->image, talker_msg_compressed.data, compression_params);

  // measure elapsed time - compression
  end1 = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
  log_time_delay1 << elapsed_seconds1.count() << std::endl;
  //std::cout << "compression latency: " << elapsed_seconds1.count() << std::endl;

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  

  // image publisher - to listener
  ros::Publisher imagePublisher = n.advertise<sensor_msgs::CompressedImage>("/camera_array/cam0/image_raw/compressed", 1000);

  // subscribe for rosbag image topic
  ros::Subscriber rosbagImageSubscriber = n.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback);

  //ros::Rate loop_rate(10);

  // define key and IV once, key load only performed once for AEGIS
  // IV is only defined at talker side

  u8 key[16] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
               	      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  aegis_state cs;
  aegis_load_key(&cs, (u32*)key);

  u8 iv[IV_SIZE] = {0x10, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 
		     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  u8 tag[TAG_SIZE] = {0};

  while (ros::ok())
  {

    // ** PART 1:  listen for ROS messages from rosbag. Then compress, encrypt and send to listener node

     // ** AUTHENTICATED ENCRYPTION **

    start2 = std::chrono::system_clock::now();

    sensor_msgs::CompressedImage talker_msg_compressed_copy;
    talker_msg_compressed_copy = talker_msg_compressed;
    
    // define data size
    int pt_length = talker_msg_compressed.data.size();

    
    if(pt_length > 0){

      // resize to include iv and tag  
      talker_msg_compressed_copy.data.resize(IV_SIZE + pt_length + TAG_SIZE);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_compressed_copy.data[0], iv, IV_SIZE);    

      // autenticated encryption
      aegis_encrypt_packet(&cs, &talker_msg_compressed_copy.data[IV_SIZE], &talker_msg_compressed_copy.data[IV_SIZE+pt_length], &talker_msg_compressed.data[0], iv, (u32*)iv, IV_SIZE, pt_length);

      // increment for unique iv
      iv[3]++;


      // measure elapsed time - encryption
      end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
      log_time_delay2 << elapsed_seconds2.count() << std::endl;
      //std::cout << "encryption latency: " << elapsed_seconds2.count() << std::endl;

      // publish compressed and encrypted image
      imagePublisher.publish(talker_msg_compressed_copy);

    }
    
    //loop_rate.sleep();
    ros::spinOnce();
  }

  log_time_delay1.close();
  log_time_delay2.close();


  return 0;
}