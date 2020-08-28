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
#include "aegis_128.h"
#include "encoder.h"


// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);

#define TAG_SIZE 16


// create a container for the data received from rosbag and listener
sensor_msgs::PointCloud2 talker_msg;


void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg = *msg;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;


  // point cloud publisher - from talker
  ros::Publisher encryptedPointcloudPublisher = n.advertise<sensor_msgs::PointCloud2>("/encrypted_pointcloud_from_talker", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber rosbagPointcloudSubscriber = n.subscribe("/os1_cloud_node/points", 1000, lidarCallback);


  // define key and IV once, key load only performed once for AEGIS
  // IV is only defined at talker side

  u8 key[16] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
               	      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  aegis_state cs;
  aegis_load_key(&cs, (u32*)key);

  u8 iv[IV_SIZE] = {0x10, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 
		     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  u8 tag[TAG_SIZE] = {0};


  while (ros::ok()){

    // ** PART 1:  listen for ROS messages from rosbag, then encrypt and send to talker node

    
    // ** AUTHENTICATED ENCRYPTION **

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;

    // define data size
    int pt_length = talker_msg.data.size();


    if(pt_length > 0){

      // resize to include iv and tag  
      talker_msg_copy.data.resize(IV_SIZE + pt_length + TAG_SIZE);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_copy.data[0], iv, IV_SIZE);    

      // autenticated encryption
      aegis_encrypt_packet(&cs, &talker_msg_copy.data[IV_SIZE], &talker_msg_copy.data[IV_SIZE+pt_length], &talker_msg.data[0], iv, (u32*)iv, IV_SIZE, pt_length);

      // increment for unique iv
      iv[3]++;

      // measure elapsed time - encryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;

      // publish encrypted point cloud
      encryptedPointcloudPublisher.publish(talker_msg_copy);  
    }
 
    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
