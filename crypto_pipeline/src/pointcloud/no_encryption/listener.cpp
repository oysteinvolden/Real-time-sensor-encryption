// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

// point cloud
#include <sensor_msgs/PointCloud2.h>

// general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>
#include <thread>


// Create a container for the data received from talker
sensor_msgs::PointCloud2 listener_msg;


void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  listener_msg = *msg;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // point cloud publisher - from listener
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/no_encrypted_points_from_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber sub = n.subscribe("/no_encrypted_points_from_talker", 1000, lidarCallback);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node and send a copy directly back to talker

    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    // publish encrypted point cloud
    lidar_pub.publish(listener_msg_copy);

    ros::spinOnce();
    
  }

  return 0;
}