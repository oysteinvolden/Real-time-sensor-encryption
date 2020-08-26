// ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h> // to construct Pointcloud2 object
#include <sensor_msgs/point_cloud2_iterator.h> // to resize Pointcloud2 object

// general
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
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/recovered_points_listener", 1000);

  // point cloud subscriber - from talker
  ros::Subscriber encryptedPointCloud = n.subscribe("/encrypted_points_from_talker", 1000, lidarCallback);

  u8 a_key[HMAC_KEYLENGTH] = {0};
  u8 e_key[AES_BLOCKSIZE] = {0};

  // Instantiate and initialize a HMAC struct
  hmac_state a_cs;
  hmac_load_key(&a_cs, a_key, HMAC_KEYLENGTH);

  // Create decryption object
  hc128_state d_cs;


  while (ros::ok()){


    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker
 
    // ** RECOVER **

    // start time - decryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::PointCloud2 listener_msg_copy;
    listener_msg_copy = listener_msg;

    // define data size
    int size_cloud = listener_msg.data.size() - TAGSIZE - HC128_IV_SIZE;
 

    if(size_cloud > 0){
      
      // Validate the tag over the IV and the ciphertext. If the(IV || Ciphertext, Tag)-pair is
      // not valid, the ciphertext is NOT decrypted.
      if ( !(tag_validation(&a_cs, &listener_msg.data[HC128_IV_SIZE+size_cloud], &listener_msg.data[0], HC128_IV_SIZE+size_cloud, TAGSIZE)) ) {
	      std::cout << "Invalid tag!" << std::endl;
      }     

      // resize to original size without tag and iv
      listener_msg_copy.data.resize(size_cloud);

      // Initialize cipher with new IV. The IV sits at the front of the msg2.
      hc128_initialize(&d_cs, (u32*)e_key, (u32*)&listener_msg.data[0]);

      // Decrypt. The ciphertext sits after the IV in msg2.
      hc128_process_packet(&d_cs, &listener_msg_copy.data[0], &listener_msg.data[HC128_IV_SIZE], size_cloud);

      // measure elapsed time - decryption operation
      end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
      log_time_delay << elapsed_seconds1.count() << std::endl;
      
      // publish recovered point cloud 
      lidar_pub.publish(listener_msg_copy);  
    }
      

    ros::spinOnce();
    
  }

  log_time_delay.close();

  return 0;
}
