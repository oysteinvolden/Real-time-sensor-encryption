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

//crypto
#include "aes_cfb.h"

#define BLOCKSIZE 16


// Create a container for the data received from talker
sensor_msgs::PointCloud2 listener_msg;


void lidarCallback2(const sensor_msgs::PointCloud2ConstPtr& msg){

  listener_msg = *msg;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // point cloud publisher - from listener
  ros::Publisher lidar_pub2 = n.advertise<sensor_msgs::PointCloud2>("/encrypted_points_from_listener", 1000);

  // point cloud publisher - from listener
  ros::Publisher lidar_pub3 = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber encryptedPointCloud = n.subscribe("/encrypted_points_from_talker", 1000, lidarCallback2);

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker

    // define data size
    int size_cloud = listener_msg.row_step * listener_msg.height;

    // RECOVER
    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;
  
    u8 key[BLOCKSIZE] = {0};
    u32 iv[BLOCKSIZE/4] = {0};
    cipher_state d_cs;
	  cfb_initialize_cipher(&d_cs, key, iv);
	  cfb_process_packet(&d_cs, &listener_msg.data[0], &listener_msg_copy.data[0], size_cloud, DECRYPT);

    // uncomment to publish decrypted point cloud on listener side 
    //lidar_pub3.publish(listener_msg_copy);

    // ENCRYPT 
    sensor_msgs::PointCloud2 listener_msg_copy2;
    listener_msg_copy2 = listener_msg_copy;
    cipher_state e_cs;
	  cfb_initialize_cipher(&e_cs, key, iv);
    cfb_process_packet(&e_cs, &listener_msg_copy.data[0], &listener_msg_copy2.data[0], size_cloud, ENCRYPT);

    // publish encrypted point cloud
    lidar_pub2.publish(listener_msg_copy2);

    ros::spinOnce();
    
  }

  return 0;
}