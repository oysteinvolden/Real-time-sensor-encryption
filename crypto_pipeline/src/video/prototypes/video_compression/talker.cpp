//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>

// opencv
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cv_bridge/cv_bridge.h>


//general
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>
#include <vector>


// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

std::vector<int> compression_params;

// create containers for the data received from rosbag 
sensor_msgs::CompressedImage talker_msg_compressed;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  // start time
  //start = std::chrono::system_clock::now();

  // define header and format
  talker_msg_compressed.header = msg->header;
  talker_msg_compressed.format = "png";

  // bridge ros message to opencv
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // compression
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); // other options: jpeg etc. CV_IMWRITE_JPEG_QUALITY, CV_IMWRITE_PNG_COMPRESSION
  compression_params.push_back(3); // level 0 - 9 - tradeoff between file size and compression latency (the higher the latency, the lower the size)

  start = std::chrono::system_clock::now();

  // store compressed opencv image to outbut buffer (ros topic)
  cv::imencode(".png", cv_ptr->image, talker_msg_compressed.data, compression_params);

  // measure elapsed time
  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "compression latency: " << elapsed_seconds.count() << std::endl;

}





int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  //image_transport::ImageTransport it(n);
  //image_transport::TransportHints hints("compressed");
  //it.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback, ros::VoidPtr(), hints);

  // image publisher - to listener
  ros::Publisher imagePublisher = n.advertise<sensor_msgs::CompressedImage>("/camera_array/cam0/image_raw/compressed", 1000);

  // subscribe for rosbag image topic
  ros::Subscriber rosbagImageSubscriber = n.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback);

  ros::Rate loop_rate(50);


  while (ros::ok())
  {

    // ** PART 1:  listen for ROS messages from rosbag, then send to talker node  
    
    // publish compressed image
    imagePublisher.publish(talker_msg_compressed);

    loop_rate.sleep();
    ros::spinOnce();
  }
  


  return 0;
}