//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

//general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


//crypto
#include "chacha.h"
#include "encoder.h"


// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);

// create a container for the data received from rosbag and listener
sensor_msgs::PointCloud2 talker_msg;


void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg = *msg;

}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedPointcloudPublisher = n.advertise<sensor_msgs::PointCloud2>("/encrypted_pointcloud_from_talker", 1000);

  // subscribe for rosbag image topic
  ros::Subscriber rosbagPointcloudSubscriber = n.subscribe("/os1_cloud_node/points", 1000, lidarCallback);



  while (ros::ok())
  {

    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node
   
    // ** ENCRYPTION **

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;
  
    // define data size
    int size = talker_msg.data.size(); 

    // Key and nonce in byte array order.
	  std::string key_string = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
	  std::string nonce_string = "000000000000004A00000000";

	  u8 key[32];
	  u8 nonce[12];

	  // Convert hex key and nonce to u8
	  hex2stringString(key, key_string.data(), key_string.size());
	  hex2stringString(nonce, nonce_string.data(), nonce_string.size());   

    
    // Initialize the cipher to encrypt
	  chacha_state e_cs;
	  chacha20_initialize(&e_cs, (u32*)key, (u32*)nonce);
  
    chacha20_process_packet(&e_cs, &talker_msg_copy.data[0], &talker_msg.data[0], size);

    // measure elapsed time - encryption operation
    end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    if(size != 0){
      log_time_delay << elapsed_seconds1.count() << std::endl;
    }
    
    encryptedPointcloudPublisher.publish(talker_msg_copy);
    
    ros::spinOnce();

  }
  
  log_time_delay.close();

  return 0;
}
