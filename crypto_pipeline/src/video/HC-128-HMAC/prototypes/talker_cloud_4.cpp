//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//openCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//general
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>
#include <thread>

//crypto
#include "hc128.h"
#include "encoder.h"
#include "hmac.h"
#include "aes_cfb.h"

// We will use the standard 128-bit HMAC-tag.
#define TAGSIZE 16

// measure RTT
std::chrono::time_point<std::chrono::system_clock> start, end;

// create a container for the data received from listener
sensor_msgs::Image talker_msg_from_list;


void cameraCallback(const sensor_msgs::ImageConstPtr& msg){

  talker_msg_from_list = *msg;

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

   // encrypted image publisher
  ros::Publisher encryptedImagePublisher = n.advertise<sensor_msgs::Image>("/encrypted_stream_from_talker", 1000);

  // recovereed image publisher
  ros::Publisher recoveredImagePublisher2 = n.advertise<sensor_msgs::Image>("/recovered_stream_talker", 1000);

  // subscribe for encrypted image sent back  
  ros::Subscriber encryptedImageSubscriber2 = n.subscribe("/encrypted_stream_from_listener", 1000, cameraCallback);


  // option 1: read image 
  /*
  std::string imagePath = "/home/oysteinvolden/catkin_ws_crypto/src/data/beginner_tutorials/src/bird.jpg";
  cv::Mat image, greyImage;
	image = cv::imread(imagePath, cv::IMREAD_COLOR);
  // convert to greyscale
	cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);
  */

  // option 2: read video
  cv::VideoCapture cap("/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/data/file_example_AVI_1280_1_5MG.avi");

  if(!cap.isOpened()){
   std::cout << "not opened" << std::endl;
   return -1;
  }


  while (ros::ok())
  {

    // start time
    start = std::chrono::system_clock::now();
    
    // ** PART 1: read video, then encrypt and send to talker node
    cv::Mat frame, greyImage;
    cap >> frame; // get a new frame from video stream
    if(frame.empty()){
      exit(1);
    }
    cv::cvtColor(frame, greyImage, cv::COLOR_BGR2GRAY);

    int size = greyImage.total() * greyImage.elemSize();
    int total_size = (HC128_IV_SIZE) + (greyImage.total() * greyImage.elemSize()) + (TAGSIZE);

    // convert Opencv image data to ros image type, take a copy and extend data field to add IV and tag
    sensor_msgs::Image talker_msg;
    cv_bridge::CvImage cv_talker_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", greyImage);
    cv_talker_msg.toImageMsg(talker_msg);

    //sensor_msgs::Image talker_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", greyImage).toImageMsg();
    sensor_msgs::Image talker_msg_copy;
    talker_msg_copy = talker_msg;
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

    // publish decrypted image with tag and iv
    encryptedImagePublisher.publish(talker_msg_copy);

	  


    // ** PART3: listen for received ROS messages from listener node, then decrypt and show recovered video **

    int size2 = talker_msg_from_list.data.size() - TAGSIZE - HC128_IV_SIZE;

    if(size2 > 0){

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
      sensor_msgs::Image talker_msg_from_list_copy;
      talker_msg_from_list_copy = talker_msg_from_list;
      talker_msg_from_list_copy.data.resize(size2);
     
      // Initialize cipher with new IV. The IV sits at the front of the msg.
	    hc128_initialize(&d_cs2, (u32*)e_key2, (u32*)&talker_msg_from_list.data[0]);

	    // Decrypt. The ciphertext sits after the IV 
      hc128_process_packet(&d_cs2, &talker_msg_from_list_copy.data[0], &talker_msg_from_list.data[HC128_IV_SIZE], size2);
 
      // publish recovered video stream
      recoveredImagePublisher2.publish(talker_msg_from_list_copy);

    }

    // measure elapsed time
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "RTT: " << elapsed_seconds.count() << std::endl;

    ros::spinOnce();
  }
  


  return 0;
}