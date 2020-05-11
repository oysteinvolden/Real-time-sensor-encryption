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

   // image publisher - to listener
  ros::Publisher imagePublisher = n.advertise<sensor_msgs::Image>("/no_encrypted_stream_from_talker", 1000);

  // image publisher - received from listener
  ros::Publisher imagePublisher2 = n.advertise<sensor_msgs::Image>("/recovered_stream_talker", 1000);

  // subscribe for image sent back from listener  
  ros::Subscriber imageSubscriber = n.subscribe("/no_encrypted_stream_from_listener", 1000, cameraCallback);


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
    exit(1);
  }

  ros::Rate loop_rate(50);

  while (ros::ok())
  {

    // start time
    start = std::chrono::system_clock::now();
    
    // ** PART 1: read video, then send to talker node

    cv::Mat frame, greyImage;
    cap >> frame; // get a new frame from video stream
    if(frame.empty()){
      exit(1);
    }
    cv::cvtColor(frame, greyImage, cv::COLOR_BGR2GRAY);

    // convert Opencv image data to ros image type and make a copy 
    sensor_msgs::Image talker_msg;
    cv_bridge::CvImage cv_talker_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", greyImage);
    cv_talker_msg.toImageMsg(talker_msg);

    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;

    // publish image
    imagePublisher.publish(talker_msg_copy);

	
    // ** PART3: listen for received ROS messages from listener node, then copy and publish video **

    // copy incomming message 
    sensor_msgs::Image talker_msg_from_list_copy;
    talker_msg_from_list_copy = talker_msg_from_list;
      
    // publish recovered video stream
    imagePublisher2.publish(talker_msg_from_list_copy);

    // measure elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "RTT: " << elapsed_seconds.count() << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }
  


  return 0;
}