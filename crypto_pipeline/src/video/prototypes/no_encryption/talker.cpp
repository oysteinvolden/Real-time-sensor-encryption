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


// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

// create a container for the data received from rosbag and listener
sensor_msgs::Image talker_msg;
sensor_msgs::Image talker_msg_from_list;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  talker_msg = *msg;

}

void cameraCallback2(const sensor_msgs::ImageConstPtr& msg){

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

  // subscribe for rosbag image topic
  ros::Subscriber rosbagImageSubscriber = n.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback);

  // subscribe for image sent back from listener  
  ros::Subscriber imageSubscriber = n.subscribe("/no_encrypted_stream_from_listener", 1000, cameraCallback2);

  ros::Rate loop_rate(50);


  while (ros::ok())
  {

    // start time
    start = std::chrono::system_clock::now();
    
    // ** PART 1:  listen for ROS messages from rosbag, then send to talker node

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

    loop_rate.sleep();
    ros::spinOnce();
  }
  


  return 0;
}