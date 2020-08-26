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
const char *path_log="time_delay_listener.txt";
std::ofstream log_time_delay(path_log);

#define TAG_SIZE 16


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


  // define key and IV once, key load only performed once for AEGIS
  // IV is only initialized at talker side

  u8 key[16] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
               	      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  aegis_state cs;
  aegis_load_key(&cs, (u32*)key);

  // initialize buffer for received iv - assume size is known
  u8 iv[IV_SIZE] = {0};
  u8 tag[TAG_SIZE] = {0};


  while (ros::ok()){

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **
    
    // ** RECOVER **

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    int pt_length = listener_msg.data.size() - IV_SIZE - TAG_SIZE;

    if(pt_length > 0){

       // the front of the message received from talker is loaded to iv
      std::memcpy(iv, &listener_msg.data[0], IV_SIZE);

      // resize to original size without iv
      listener_msg_copy.data.resize(pt_length);

      // authenticted decryption    
      if (!aegis_decrypt_packet(&cs, &listener_msg_copy.data[0], &listener_msg.data[IV_SIZE], iv, (u32*)iv, (u32*)&listener_msg.data[IV_SIZE+pt_length], IV_SIZE, pt_length))
	    {
		    std::cout << "Invalid tag!\n";
		    exit(1);
	    }

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
