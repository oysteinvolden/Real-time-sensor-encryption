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
#include "sosemanuk.h"
#include "encoder.h"


// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

// create a container for the data received from listener
sensor_msgs::Image talker_msg_from_list;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  talker_msg_from_list = *msg;

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_talker", 1000);

  // recovereed image publisher
  ros::Publisher recoveredImagePublisher2 = n.advertise<sensor_msgs::Image>("/recovered_stream_talker", 1000);

  // subscribe for encrypted image sent back  
  ros::Subscriber encryptedImageSubscriber2 = n.subscribe("/encrypted_stream_from_listener", 1000, cameraCallback);


  // option 1: read image 
  /*
  std::string imagePath = "/home/oysteinvolden/catkin_ws_crypto/src/data/beginner_tutorials/src/bird.jpg";
  cv::Mat image, greyImage;
	image = cv::imread(imagePath, cv::IMREAD_COLOR);
  // convert to greyscale
	cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);
  */

  // option 2: read video
  cv::VideoCapture cap("/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/data/file_example_AVI_1280_1_5MG.avi");

  if(!cap.isOpened()){
   std::cout << "not opened" << std::endl;
   return -1;
  }


  while (ros::ok())
  {

    // start time
    start = std::chrono::system_clock::now();
    
    // ** PART 1: read video, then encrypt and send to talker node
    cv::Mat frame, greyImage;
    cap >> frame; // get a new frame from video stream
    if(frame.empty()){
      exit(1);
    }
    cv::cvtColor(frame, greyImage, cv::COLOR_BGR2GRAY);

    int size = greyImage.total() * greyImage.elemSize();

    // convert Opencv image data to ros image type, take a copy and extend data field to add IV and tag
    sensor_msgs::Image talker_msg;
    cv_bridge::CvImage cv_talker_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", greyImage);
    cv_talker_msg.toImageMsg(talker_msg);

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
      recoveredImagePublisher2.publish(talker_msg_from_list_copy);

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