//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//openCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>
#include <thread>

//crypto
#include "hc128.h"
#include "encoder.h"


// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

#define BLOCKSIZE 16

void serialize(u8 *out, cv::Mat *in, int size)
{
	for (int h = 0; h < in->rows; h++)
	{
		for (int w = 0; w < in->cols; w++)
		{
			*out = (*in).at<u8>(h,w); out++;
		}
	}
}

void deserialize(cv::Mat *out, u8 *in, int size)
{
	for (int h = 0; h < out->rows; h++)
	{
		for (int w = 0; w < out->cols; w++)
		{
			(*out).at<u8>(h,w) = *in; in++;
		}
	}
}

//define captured frame and current frame in use
cv::Mat cap_frame, cur_frame;

void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
  {
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cap_frame = cv_ptr->image.clone();

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  return;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_talker", 1000);

  // subscribe for encrypted image sent back  
  ros::Subscriber encryptedImageSubscriber2 = n.subscribe("/encrypted_stream_from_listener", 1000, cameraCallback);

  /*
  // read image 
  std::string imagePath = "/home/crypto-dev/catkin_ws_crypto/src/crypto_pipeline/src/data/bird.jpg";
  cv::Mat image, greyImage;
	image = cv::imread(imagePath, cv::IMREAD_COLOR);
  // convert to greyscale
	cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);
  */

  cv::VideoCapture cap("/home/crypto-dev/catkin_ws_crypto/src/crypto_pipeline/src/data/file_example_AVI_1280_1_5MG.avi");

  if(!cap.isOpened()){
   std::cout << "not opened" << std::endl;
   return -1;
  }


  

  while (ros::ok())
  {
    

    // ** PART 1: read video, then encrypt and send to talker node
    cv::Mat frame, greyImage;
    cap >> frame; // get a new frame from video stream
    cv::cvtColor(frame, greyImage, cv::COLOR_BGR2GRAY);


    // start time
    start = std::chrono::system_clock::now();

    // define key and IV
    std::string hexkey = "0F62B5085BAE0154A7FA4DA0F34699EC";
	  std::string hexIv = "288FF65DC42B92F960C72E95FC63CA31";    
    
    u32 key[4];
    hex2stringString((u8*)key, hexkey.data(), 32);
    u32 iv[4];
    hex2stringString((u8*)iv, hexIv.data(), 32);

    hc128_state e_cs;
	  hc128_initialize(&e_cs, key, iv);

    int size = greyImage.total() * greyImage.elemSize();
    u8 plaintext[size];

    if(greyImage.isContinuous()){
      serialize(plaintext, &greyImage, size);
    }
		  
    cv::Mat deserialized = cv::Mat::zeros(cv::Size(greyImage.cols, greyImage.rows), CV_8U);
	  deserialize(&deserialized, plaintext, size);

    u8 ciphertext[size];

	  cv::Mat encrypted = cv::Mat::zeros(cv::Size(greyImage.cols, greyImage.rows), CV_8U);
  
    hc128_process_packet(&e_cs, ciphertext, plaintext, size);

    deserialize(&encrypted, ciphertext, size);

    //cv::imshow( "Encrypted image", encrypted );
	  //cv::waitKey(500);

    // publish encrypted image via ROS
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", encrypted).toImageMsg();
    encryptedImagePublisher.publish(msg);
    	  


    // ** PART3: listen for received ROS messages from listener node, then decrypt and show recovered video **
    cur_frame = cap_frame.clone();
    if(!cur_frame.empty() && cur_frame.isContinuous()){

      int size = cur_frame.total() * cur_frame.elemSize();

      u8 ciphertext[size];
      serialize(ciphertext, &cur_frame, size);

      // RECOVER 
      hc128_state d_cs;
	    hc128_initialize(&d_cs, key, iv);

      u8 deciphered[size];

      hc128_process_packet(&d_cs, deciphered, ciphertext, size);

      cv::Mat decrypted = cv::Mat::zeros(cv::Size(cur_frame.cols, cur_frame.rows), CV_8UC1);

      deserialize(&decrypted, deciphered, size);

	    cv::imshow( "Recovered image", decrypted );
	    cv::waitKey(10);

      // measure elapsed time
      end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;
      std::cout << "RTT: " << elapsed_seconds.count() << std::endl;

    }
      
    // increment key so its unique for each cryptographical operation
    hexkey += "1";

    ros::spinOnce();
  }
  


  return 0;
}
