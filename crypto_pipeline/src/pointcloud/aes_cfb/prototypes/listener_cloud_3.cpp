//ROS libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

//point cloud
#include <sensor_msgs/PointCloud2.h>

//IO etc
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

//int seq_counter_list;

std::chrono::time_point<std::chrono::system_clock> start1, end1;


#define BLOCKSIZE 16

sensor_msgs::PointCloud2 cloud_msg2;
//sensor_msgs::PointCloud2Ptr cloud_msg2 = boost::make_shared<sensor_msgs::PointCloud2>();


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


void lidarCallback2(const sensor_msgs::PointCloud2ConstPtr& msg2){

  cloud_msg2 = *msg2;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  start1 = std::chrono::system_clock::now();

  // point cloud publisher
  ros::Publisher lidar_pub2 = n.advertise<sensor_msgs::PointCloud2>("/points2", 1000);

  // point cloud subscriber
  ros::Subscriber encryptedPointCloud = n.subscribe("/points", 1000, lidarCallback2);

  //u8 key[BLOCKSIZE] = {0};
  //u32 iv[BLOCKSIZE/4] = {0};

  //ros::Rate loop_rate(10);


  while (ros::ok()){

      // update internal sequence counter
      //seq_counter_list = cloud_msg2.header.seq;

      //log files
      const char *path_log_ciphertext_2="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_ciphertext_2.txt";
      std::ofstream log_ciphertext_2(path_log_ciphertext_2);
      const char *path_log_ciphertext_3="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_ciphertext_3.txt";
      std::ofstream log_ciphertext_3(path_log_ciphertext_3);
      const char *path_log_recovered_2="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_recovered_2.txt";
      std::ofstream log_recovered_2(path_log_recovered_2);

      //define 
      int size_cloud = cloud_msg2.row_step * cloud_msg2.height;
      u8* data_cloud2 = new u8[size_cloud];

      for(int i = 0; i < size_cloud; i++){
        data_cloud2[i] = cloud_msg2.data[i];
        char hex[3];
        string2hexString(hex, &data_cloud2[i], 2);
        std::string printableHex(hex, 2);
        //std::cout << printableHex << " ";
        if(i < 200){
          log_ciphertext_2 << printableHex << " ";
        }
    
      }

      log_ciphertext_2.close();

      u8* out_buff2 = new u8[size_cloud];
    
      memcpy(out_buff2, data_cloud2, size_cloud);

      delete[] data_cloud2;

      u8* deciphered2 = new u8[size_cloud];

      // RECOVER

      u8 key[BLOCKSIZE] = {0};
      u32 iv[BLOCKSIZE/4] = {0};
      cipher_state d_cs;
	    cfb_initialize_cipher(&d_cs, key, iv);

	    cfb_process_packet(&d_cs, out_buff2, deciphered2, size_cloud, DECRYPT);

      u8* out_buff3 = new u8[size_cloud];    
      memcpy(out_buff3, deciphered2, size_cloud);

      for(int i = 0; i < size_cloud; i++){
        char hex[3]; 
        string2hexString(hex, &deciphered2[i], 2);
        std::string printableHex(hex, 2);
        if(i < 200){
          log_recovered_2 << printableHex << " ";
        }  
      }

      delete[] deciphered2;

      log_recovered_2.close();


      // ENCRYPT 

      //u8* ciphertext = new u8[size_cloud];
    
      cipher_state e_cs;
	    cfb_initialize_cipher(&e_cs, key, iv);
      cfb_process_packet(&e_cs, out_buff3, out_buff3, size_cloud, ENCRYPT);

      //copy ciphertext
      //u8* ciphertext_copy = new u8[size_cloud];    
      //memcpy(ciphertext_copy, out_buff3, size_cloud);

      sensor_msgs::PointCloud2 cloud_msg_copy2;
      cloud_msg_copy2 = cloud_msg2;

      // update pointcloud2 ros message
      for(int i = 0; i < size_cloud; i++){
        cloud_msg_copy2.data[i] = out_buff3[i];
      }

      // publish encrypted point cloud
      lidar_pub2.publish(cloud_msg_copy2);

      //delete[] ciphertext_copy;

      /*
      // print encrypted
      for(int i = 0; i < size_cloud; i++){
        char hex[3];
        string2hexString(hex, &out_buff3[i], 2);
        std::string printableHex(hex, 2);
        if(i < 200){
          log_ciphertext_3 << printableHex << " ";
        }
      }
      */
      log_ciphertext_3.close();
      
      //delete[] ciphertext;
      delete[] out_buff3;


    ros::spinOnce();


    //measure time for a round
    end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end1 - start1;

    // if a reasonable amount of time is passed, quit the program

    std::cout << elapsed_seconds.count() << std::endl;

    if(elapsed_seconds.count() > 30){
      exit(1);
    }  
    
  }

  return 0;
}