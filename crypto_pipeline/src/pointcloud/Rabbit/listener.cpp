// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>


// general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


//crypto
#include "rabbit.h"
#include "encoder.h"


// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_listener.txt";
std::ofstream log_time_delay(path_log);


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
  ros::Publisher recoveredPointcloudPublisher = n.advertise<sensor_msgs::PointCloud2>("/recovered_pointcloud_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber encryptedPointcloudSubscriber = n.subscribe("/encrypted_pointcloud_from_talker", 1000, lidarCallback);

  // define key once, key load only performed once for rabbit
  std::string keyString = "00000000000000000000000000000000";
  u8 key[keyString.size()/2];
  hex2stringString(key, keyString.data(), keyString.size());

  rabbit_state d_cs;
  rabbit_key_setup(&d_cs, (u32*)key);

  // initialize buffer to contain iv - assume size is known
  u8 iv[16] = {0};

  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

    // ** RECOVER **

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    // copy
    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    // define recovered data size
    int size_cloud = listener_msg.data.size() - sizeof(iv)/2;

    if(size_cloud > 0){

      // the front of the message received from talker is loaded to iv
      std::memcpy(iv, &listener_msg.data[0], sizeof(iv)/2);

      // resize to original size without iv
      listener_msg_copy.data.resize(size_cloud);

      // Load key and encrypt 
      rabbit_iv_setup(&d_cs, (u32*)iv);
      rabbit_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[sizeof(iv)/2], size_cloud);

      // measure elapsed time - decryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;

      // publish recovered point cloud
      recoveredPointcloudPublisher.publish(listener_msg_copy);  
    }
 
    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
