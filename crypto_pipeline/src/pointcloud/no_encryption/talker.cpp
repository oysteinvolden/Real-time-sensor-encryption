//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

//point cloud
#include <sensor_msgs/PointCloud2.h>

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

// Create a container for the data received from rosbag and listener
sensor_msgs::PointCloud2 talker_msg; 
sensor_msgs::PointCloud2 talker_msg_from_list;


// callback for rosbag
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg = *msg;

  return;
}

// callback for listener node
void lidarCallback2(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg_from_list = *msg;

  return;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;


  // point cloud publisher - from talker
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/no_encrypted_points_from_talker", 1000);

  ros::Publisher lidar_recovered = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_talker", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/os1_cloud_node/points", 1000, lidarCallback); 

  // point cloud subscriber - from listener
  ros::Subscriber sub_list = n.subscribe<sensor_msgs::PointCloud2> ("/no_encrypted_points_from_listener", 1000, lidarCallback2); 

  // start time
  start = std::chrono::system_clock::now();

  while (ros::ok())
  {
    // ** PART 1: listen for ROS messages from rosbag, then send a copy directly to listener node

    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;

    // publish point cloud
    lidar_pub.publish(talker_msg_copy);
      
    // ** PART3: listen for received ROS messages from listener node, then publish a copied point cloud **

    sensor_msgs::PointCloud2 talker_msg_from_list_copy;
    talker_msg_from_list_copy = talker_msg_from_list;

    // publish point cloud data received from listener
    lidar_recovered.publish(talker_msg_from_list);

    // measure elapsed time (RTT when rosbag, listener and talker node is running at the same time)
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "RTT: " << elapsed_seconds.count() << std::endl;

    ros::spinOnce();
  }
  


  return 0;
}