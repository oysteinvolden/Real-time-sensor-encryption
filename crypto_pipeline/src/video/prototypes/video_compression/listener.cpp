// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

// general
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


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
  ros::Publisher imagePublisher = n.advertise<sensor_msgs::CompressedImage>("/compressed_stream_from_listener", 1000);

  ros::Rate loop_rate(50);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then send directly back to talker **
    
    // copy incomming message 
    sensor_msgs::CompressedImage listener_msg_copy;
    listener_msg_copy = listener_msg; 

    // publish compressed image
    imagePublisher.publish(listener_msg_copy);

    loop_rate.sleep();
    ros::spinOnce();
    
  }

  return 0;
}