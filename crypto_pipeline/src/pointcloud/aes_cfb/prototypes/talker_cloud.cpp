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

//point cloud
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>

//IO
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <chrono>
#include <string.h>
#include <stdio.h>

#include<thread>


//crypto
#include "/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/include/beginner_tutorials/aes_cfb.h"



void string2hexString(char* output, const unsigned char* input, int size)
{
	int loop;
	int i;

	i = 0;
	loop = 0;

	while(loop != size)
	{
	//	std::cout << "loop: " << loop << " | i = " << i << std::endl;
		sprintf((char*)(output+i), "%02X", input[loop]);
		loop+=1;
		i+=2;
	}
	output[i++] = '\0';
}


// Create a container for the data.
//sensor_msgs::PointCloud2 cloud_msg;
sensor_msgs::PointCloud2Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();


std::chrono::time_point<std::chrono::system_clock> start, end;

#define BLOCKSIZE 16

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  //std::cout << "lidar callback" << std::endl;

  *cloud_msg = *msg;

  return;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;


  // lidar publisher
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/points", 1000);

  // lidar subscriber
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/os1_cloud_node/points", 1000, lidarCallback); 


 //ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    // log files
    const char *path_log_plaintext1="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_plaintext_1.txt";
    std::ofstream log_plaintext_1(path_log_plaintext1);
    const char *path_log_plaintext2="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_plaintext_2.txt";
    std::ofstream log_plaintext_2(path_log_plaintext2);
    const char *path_log_ciphertext="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_ciphertext.txt";
    std::ofstream log_ciphertext(path_log_ciphertext);
    const char *path_log_recovered="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_recovered.txt";
    std::ofstream log_recovered(path_log_recovered);


    int size_cloud = cloud_msg->row_step * cloud_msg->height;

    u8* data_cloud = new u8[size_cloud];

    //std::this_thread::sleep_for(std::chrono::duration<int>(1));


    for(int i = 0; i < size_cloud; i++){
      *data_cloud = cloud_msg->data[i];
      //data_cloud[i] = cloud_msg->data[i];
      //char hex[3];
      //string2hexString(hex, data_cloud, 2);
      //std::string printableHex(hex, 2);
      //std::cout << printableHex << std::endl;
      //if(i < 100){
      //  log_plaintext_1 << printableHex << " ";
      //}
      data_cloud++;
    }

    //std::this_thread::sleep_for(std::chrono::duration<int>(1));

    data_cloud -= size_cloud;

    u8* out_buff = new u8[size_cloud];
    
    memcpy(out_buff, data_cloud, size_cloud);

    //data_cloud -= size_cloud;
    delete[] data_cloud;
   

    
    for(int i = 0; i < size_cloud; i++){
      char hex[3];
      string2hexString(hex, out_buff, 2);
      std::string printableHex(hex, 2);
      //std::cout << printableHex << std::endl;
      if(i < 200){
      log_plaintext_2 << printableHex << " ";
      }
      out_buff++;
    }
    
    
    out_buff-=size_cloud;
  

    //std::cout << "test3" << std::endl;
  
    //std::this_thread::sleep_for(std::chrono::duration<int>(1));

    
    // ** lidar encryption **
    
    u8 key[BLOCKSIZE] = {0};
	  u32 iv[BLOCKSIZE/4] = {0};

	  cipher_state e_cs;

    //std::cout << "test4" << std::endl;

	  cfb_initialize_cipher(&e_cs, key, iv);

    //std::cout << "test5" << std::endl;

    u8* ciphertext = new u8[size_cloud];

    //std::cout << "test6" << std::endl;

    //std::this_thread::sleep_for(std::chrono::duration<int>(1));

    cfb_process_packet(&e_cs, out_buff, ciphertext, size_cloud, ENCRYPT);


    //out_buff-=size_cloud;
    delete[] out_buff;

    
    // print ciphertext
    for(int i = 0; i < size_cloud; i++){
      //cloud_msg->data[i] = *ciphertext;
      char hex[3];
      string2hexString(hex, ciphertext, 2);
      std::string printableHex(hex, 2);
      //std::cout << printableHex << " ";
      if(i < 200){
        log_ciphertext << printableHex << " ";
      }
      ciphertext++;
    }
    
    ciphertext-=size_cloud;

    //std::cout << "test7" << std::endl;

    //std::this_thread::sleep_for(std::chrono::duration<int>(1));
   
    // update data field with encrypted point cloud data
    /*
    for(int i = 0; i < size_cloud; i++){
      cloud_msg->data[i] = *ciphertext;
      char hex[3];
      string2hexString(hex, ciphertext, 2);
      std::string printableHex(hex, 2);
      //std::cout << printableHex << " ";
      if(i < 100){
        log_recovered << printableHex << " ";
      }
      ciphertext++;
    }
    */

    // publish the same ros message as subscibed except for encrypted data field
    //lidar_pub.publish(cloud_msg);


    //std::cout << "test8" << std::endl;
    //std::this_thread::sleep_for(std::chrono::duration<int>(1));
    
    //u8* out_buff = new u8[size_cloud];
    
    //memcpy(out_buff, ciphertext, size_cloud);
   
    //delete[] ciphertext;
    //std::cout << "test9" << std::endl;
    //std::this_thread::sleep_for(std::chrono::duration<int>(1));
    

       

    // RECOVER - 

	  cipher_state d_cs;
	  cfb_initialize_cipher(&d_cs, key, iv);

    u8* deciphered = new u8[size_cloud];

	  cfb_process_packet(&d_cs, ciphertext, deciphered, size_cloud, DECRYPT);

    //ciphertext-=size_cloud; 
    delete[] ciphertext;    
    
    for(int i = 0; i < size_cloud; i++){
      char hex[3];
      string2hexString(hex, deciphered, 2);
      std::string printableHex(hex, 2);
      //std::cout << printableHex << " ";
      if(i < 200){
        log_recovered << printableHex << " ";
      }
      deciphered++;   
    }
    

    //std::cout << "test 11" << std::endl;
    //ciphertext-=size_cloud;
    //delete[] ciphertext;
    //std::this_thread::sleep_for(std::chrono::duration<int>(1));
    //std::cout << "test 12" << std::endl;
    deciphered-=size_cloud;
    delete[] deciphered;
    
  
    
    //log_plaintext_1.close();
    log_plaintext_2.close();
    log_ciphertext.close();
    log_recovered.close();


    ros::spinOnce();
    //loop_rate.sleep();
    
  }
  


  return 0;
}