//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>

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
#include "aes_cfb.h"

// We will use the standard 128-bit HMAC-tag.
#define TAGSIZE 16

// measure delay
std::chrono::time_point<std::chrono::system_clock> start1, end1, start2, end2;

// log time delay
const char *path_log="time_delay_talker.txt";
std::ofstream log_time_delay(path_log);


// create a container for the data received from rosbag and listener
sensor_msgs::Image talker_msg;
//sensor_msgs::Image talker_msg_from_list;

void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  talker_msg = *msg;

}

/*
void cameraCallback2(const sensor_msgs::ImageConstPtr& msg){

  talker_msg_from_list = *msg;

}
*/



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_talker", 1000);

  // recovereed image publisher
  //ros::Publisher recoveredImagePublisher2 = n.advertise<sensor_msgs::Image>("/recovered_stream_talker", 1000);

  // subscribe for rosbag image topic
  ros::Subscriber rosbagImageSubscriber = n.subscribe("/camera_array/cam0/image_raw", 1000, cameraCallback);

  // subscribe for encrypted image sent back  
  //ros::Subscriber encryptedImageSubscriber2 = n.subscribe("/encrypted_stream_from_listener", 1000, cameraCallback2);


  while (ros::ok())
  {

    // ** PART 2: listen for received ROS messages from talker node, then decrypt and encrypt before sending back to talker **

    // ** RECOVER **

    // start time - encryption
    start1 = std::chrono::system_clock::now();

    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;
    
    // define data size and resize to add tag and iv
    int size = talker_msg.data.size();
    int total_size = (HC128_IV_SIZE) + (talker_msg.data.size()) + (TAGSIZE);
    
    talker_msg_copy.data.resize(total_size);

    u8 a_key[HMAC_KEYLENGTH] = {0};
    u8 e_key[AES_BLOCKSIZE] = {0};
    u32 iv[AES_BLOCKSIZE/4] = {0};

    // Instantiate and initialize a HMAC struct
    hmac_state a_cs;
    hmac_load_key(&a_cs, a_key, HMAC_KEYLENGTH);

    hc128_state e_cs;
    hc128_initialize(&e_cs, (u32*)e_key, iv);

    // Load the IV
    std::memcpy(&talker_msg_copy.data[0], iv, HC128_IV_SIZE);    

    //encrypt
    hc128_process_packet(&e_cs, &talker_msg_copy.data[HC128_IV_SIZE], &talker_msg.data[0], size);

    // Compute the tag and append. NB! Tag is computed over IV || Ciphertext
    tag_generation(&a_cs, &talker_msg_copy.data[HC128_IV_SIZE+size], &talker_msg_copy.data[0], HC128_IV_SIZE+size, TAGSIZE); 

    // measure elapsed time - encryption
    end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    if(size != 0){
      log_time_delay << elapsed_seconds1.count() << std::endl;
    }   

    // publish decrypted image with tag and iv
    encryptedImagePublisher.publish(talker_msg_copy);


    // ** PART3: listen for received ROS messages from listener node, then decrypt and show recovered video **
    /*
    // start time - decryption
    start2 = std::chrono::system_clock::now();

    int size2 = talker_msg_from_list.data.size() - TAGSIZE - HC128_IV_SIZE;

    if(size2 > 0){

      sensor_msgs::Image talker_msg_from_list_copy;
      talker_msg_from_list_copy = talker_msg_from_list;

      // RECOVER 
      u8 a_key2[HMAC_KEYLENGTH] = {0};
      u8 e_key2[AES_BLOCKSIZE] = {0};

      // Instantiate and initialize a HMAC struct
      hmac_state a_cs2;
      hmac_load_key(&a_cs2, a_key2, HMAC_KEYLENGTH);

      // Validate the tag over the IV and the ciphertext. If the(IV || Ciphertext, Tag)-pair is
	    // not valid, the ciphertext is NOT decrypted.
      if ( !(tag_validation(&a_cs2, &talker_msg_from_list.data[HC128_IV_SIZE+size2], &talker_msg_from_list.data[0], HC128_IV_SIZE+size2, TAGSIZE)) ) {
	      std::cout << "Invalid tag!" << std::endl;
      }
      else
      {
        // Else, tag is valid. Proceed to initialize the cipher and decrypt.
	      std::cout << "Valid tag!\n" << std::endl;
      }

      // Create decryption object
	    hc128_state d_cs2;

      // copy incomming message and resize to original size without tag and iv
      talker_msg_from_list_copy.data.resize(size2);
     
      // Initialize cipher with new IV. The IV sits at the front of the msg.
	    hc128_initialize(&d_cs2, (u32*)e_key2, (u32*)&talker_msg_from_list.data[0]);

	    // Decrypt. The ciphertext sits after the IV 
      hc128_process_packet(&d_cs2, &talker_msg_from_list_copy.data[0], &talker_msg_from_list.data[HC128_IV_SIZE], size2);

      // measure elapsed time - decryption
      end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
      log_time_delay << elapsed_seconds2.count() << std::endl;
      
 
      // publish recovered video stream
      recoveredImagePublisher2.publish(talker_msg_from_list_copy);

    }
    */

    ros::spinOnce();
  }
  
  log_time_delay.close();


  return 0;
}
