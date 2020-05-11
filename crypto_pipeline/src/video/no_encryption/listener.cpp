// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>


// general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


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
  ros::Subscriber imageSubscriber = n.subscribe("/no_encrypted_stream_from_talker", 1000, cameraCallback);

  // encrypted image publisher
  ros::Publisher imagePublisher = n.advertise<sensor_msgs::Image>("/no_encrypted_stream_from_listener", 1000);

  ros::Rate loop_rate(50);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then send directly back to talker **
    
    // copy incomming message 
    sensor_msgs::Image listener_msg_copy;
    listener_msg_copy = listener_msg; 

    // publish encrypted image with tag and iv
    imagePublisher.publish(listener_msg_copy);


    ros::spinOnce();
    loop_rate.sleep();
    
  }

  return 0;
}