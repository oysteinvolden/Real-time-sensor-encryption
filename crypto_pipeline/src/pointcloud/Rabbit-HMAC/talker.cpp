//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

//general
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>


//crypto
#include "rabbit.h"
#include "encoder.h"
#include "hmac.h"
//#include "aes_cfb.h"

// We will use the standard 128-bit HMAC-tag.
#define TAGSIZE 16

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

   // point cloud publisher - from talker
  ros::Publisher encryptedPointcloudPublisher = n.advertise<sensor_msgs::PointCloud2>("/encrypted_pointcloud_from_talker", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber rosbagPointcloudSubscriber = n.subscribe("/os1_cloud_node/points", 1000, lidarCallback);

  
  // Instantiate and initialize a HMAC struct
  u8 a_key[HMAC_KEYLENGTH] = {0};
  hmac_state a_cs;
  hmac_load_key(&a_cs, a_key, HMAC_KEYLENGTH);

  // define key and IV once, key load only performed once for rabbit
  // IV is only defined at talker side
  std::string keyString = "00000000000000000000000000000000";
  u8 key[keyString.size()/2];
  hex2stringString(key, keyString.data(), keyString.size());
  rabbit_state cs;
  rabbit_key_setup(&cs, (u32*)key);
   
  std::string ivString = "597E26C175F573C3";
  u8 iv[ivString.size()/2];
  hex2stringString(iv, ivString.data(), ivString.size());


  while (ros::ok())
  {

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

    // ** RECOVER **

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;
    
    // define data size and resize to add tag and iv
    int size = talker_msg.data.size();
    
    if(size > 0){

      // resize to add iv and tag 
      talker_msg_copy.data.resize(ivString.size()/2 + size + TAGSIZE);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_copy.data[0], iv, ivString.size()/2);   

      // Load iv and encrypt 
      rabbit_iv_setup(&cs, (u32*)iv);
      rabbit_process_packet(&cs, &talker_msg_copy.data[ivString.size()/2], &talker_msg.data[0], size);

      // Compute the tag and append. NB! Tag is computed over IV || Ciphertext
      tag_generation(&a_cs, &talker_msg_copy.data[ivString.size()/2+size], &talker_msg_copy.data[0], ivString.size()/2+size, TAGSIZE); 

      // increment for unique iv
      iv[3]++;

      // measure elapsed time - encryption
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;

      // publish encrypted image with tag and iv
      encryptedPointcloudPublisher.publish(talker_msg_copy);

    }   
  
    ros::spinOnce();

  }
  
  log_time_delay.close();


  return 0;
}
