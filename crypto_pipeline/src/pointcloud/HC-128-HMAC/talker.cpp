//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>  // to construct Pointcloud2 object
#include <sensor_msgs/point_cloud2_iterator.h> // to resize Pointcloud2 object

//general
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>

//crypto
#include "hc128.h"
#include "encoder.h"
#include "hmac.h"


// We will use the standard 128-bit HMAC-tag.
#define TAGSIZE 16

#define AES_BLOCKSIZE 16

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);

// Create a container for the data received from rosbag and listener
sensor_msgs::PointCloud2 talker_msg; 


// callback for rosbag
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  talker_msg = *msg;

}




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  // point cloud publisher - from talker
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/encrypted_points_from_talker", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/os1_cloud_node/points", 1000, lidarCallback); 


  u8 a_key[HMAC_KEYLENGTH] = {0};
  u8 e_key[AES_BLOCKSIZE] = {0};
  u32 iv[AES_BLOCKSIZE/4] = {0};

  // Instantiate and initialize a HMAC struct
  hmac_state a_cs;
  hmac_load_key(&a_cs, a_key, HMAC_KEYLENGTH);

  // create encryption object
  hc128_state e_cs;

  while (ros::ok())
  {
    
    // ** PART 1: listen for ROS messages from rosbag, then encrypt and send to talker node

    // ** ENCRYPTION **
    
    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 talker_msg_copy;
    talker_msg_copy = talker_msg;

    // define data size
    int size_cloud = talker_msg.data.size();
   

    if(size_cloud > 0){

      // resize to add iv and tag 
      talker_msg_copy.data.resize(HC128_IV_SIZE + size_cloud + TAGSIZE);

      // Load the IV to the front of the message
      std::memcpy(&talker_msg_copy.data[0], iv, HC128_IV_SIZE);

      // initialize and encrypt
      hc128_initialize(&e_cs, (u32*)e_key, iv);
      hc128_process_packet(&e_cs, &talker_msg_copy.data[HC128_IV_SIZE], &talker_msg.data[0], size_cloud);

      // Compute the tag and append. NB! Tag is computed over IV || Ciphertext
      tag_generation(&a_cs, &talker_msg_copy.data[HC128_IV_SIZE+size_cloud], &talker_msg_copy.data[0], HC128_IV_SIZE+size_cloud, TAGSIZE);

      // increment for unique iv
      iv[3]++;

      // measure elapsed time - encryption operation
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;
      
      // publish decrypted point cloud with tag and iv
      lidar_pub.publish(talker_msg_copy);  
    }

    ros::spinOnce();

  }

  log_time_delay.close();

  return 0;
}
