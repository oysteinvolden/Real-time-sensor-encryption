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
#include <thread>

//crypto
#include "/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/include/beginner_tutorials/aes_cfb.h"

int seq_counter1, seq_counter2;

std::chrono::time_point<std::chrono::system_clock> start, end;


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


// Create a container for the data received from rosbag
sensor_msgs::PointCloud2 cloud_msg;
//sensor_msgs::PointCloud2Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();

// Create a container for the data received from listener
sensor_msgs::PointCloud2 cloud_msg_list;


#define BLOCKSIZE 16

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

  cloud_msg = *msg;

  return;
}

void lidarCallback2(const sensor_msgs::PointCloud2ConstPtr& msg){

  cloud_msg_list = *msg;

  return;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  // point cloud publisher
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/points", 1000);

  ros::Publisher lidar_recovered = n.advertise<sensor_msgs::PointCloud2>("/recovered_points", 1000);

  // point cloud subscriber - from rosbag
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/os1_cloud_node/points", 1000, lidarCallback); 

  // point cloud subscriber - from listener
  ros::Subscriber sub_list = n.subscribe<sensor_msgs::PointCloud2> ("/points2", 1000, lidarCallback2); 

  // measure time
  start1 = std::chrono::system_clock::now();


  while (ros::ok())
  {

      seq_counter1 = cloud_msg.header.seq;

      // log files
      const char *path_log_plaintext1="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_plaintext_1.txt";
      std::ofstream log_plaintext_1(path_log_plaintext1);
      const char *path_log_ciphertext_1="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_ciphertext_1.txt";
      std::ofstream log_ciphertext_1(path_log_ciphertext_1);

      // define data field
      int size_cloud = cloud_msg.row_step * cloud_msg.height;
      u8* data_cloud = new u8[size_cloud];


      for(int i = 0; i < size_cloud; i++){
        data_cloud[i] = cloud_msg.data[i];
        char hex[3];
        string2hexString(hex, &data_cloud[i], 2);
        std::string printableHex(hex, 2);
        if(i < 200){
          log_plaintext_1 << printableHex << " ";
        }
      }

      log_plaintext_1.close();


      // ** ENCRYPTION **

      u8 key[BLOCKSIZE] = {0};
	    u32 iv[BLOCKSIZE/4] = {0};
      cipher_state e_cs;
	    cfb_initialize_cipher(&e_cs, key, iv);
      cfb_process_packet(&e_cs, data_cloud, data_cloud, size_cloud, ENCRYPT);

      // copy original message and overwrite data field with encrypted values
      sensor_msgs::PointCloud2 cloud_msg_copy;
      cloud_msg_copy = cloud_msg;
      for(int i = 0; i < size_cloud; i++){
        cloud_msg_copy.data[i] = data_cloud[i];
      }
    
      // publish encrypted point cloud
      lidar_pub.publish(cloud_msg_copy);

      

      //print ciphertext
      for(int i = 0; i < size_cloud; i++){
        char hex[3];
        string2hexString(hex, &data_cloud[i], 2);
        std::string printableHex(hex, 2);
        if(i < 200){
          log_ciphertext_1 << printableHex << " ";
        }
      }

      log_ciphertext_1.close();
      
      delete[] data_cloud;




      // ** RECOVER: listen for received ROS messages from listener node **

      std::cout << "seq 1: " << seq_counter1 << std::endl;

        seq_counter2 = cloud_msg_list.header.seq;

        std::cout << "seq 2: " << seq_counter2 << std::endl;

        const char *path_log_ciphertext_4="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_ciphertext_4.txt";
        std::ofstream log_ciphertext_4(path_log_ciphertext_4);
        const char *path_log_recovered_3="/home/oysteinvolden/catkin_ws_crypto/src/beginner_tutorials/src/log_recovered_3.txt";
        std::ofstream log_recovered_3(path_log_recovered_3);

        int size_cloud2 = cloud_msg_list.row_step * cloud_msg_list.height;
        u8* ciphertext_2 = new u8[size_cloud2];
      
        for(int i = 0; i < size_cloud2; i++){
          ciphertext_2[i] = cloud_msg_list.data[i];
          char hex[3];
          string2hexString(hex, &ciphertext_2[i], 2);
          std::string printableHex(hex, 2);
          if(i < 200){
            log_ciphertext_4 << printableHex << " ";
          }
        }
      

        log_ciphertext_4.close();

        //copy ciphertext
        u8* ciphertext_3 = new u8[size_cloud2];    
        memcpy(ciphertext_3, ciphertext_2, size_cloud2);

        delete[] ciphertext_2;

        
        // RECOVER 
        cipher_state d_cs;
	      cfb_initialize_cipher(&d_cs, key, iv);

        u8* deciphered_3 = new u8[size_cloud2];

	      cfb_process_packet(&d_cs, ciphertext_3, deciphered_3, size_cloud2, DECRYPT);

        // copy recovered
        u8* deciphered_4 = new u8[size_cloud2];    
        memcpy(deciphered_4, deciphered_3, size_cloud2);

 
        delete[] ciphertext_3;    
    
        for(int i = 0; i < size_cloud2; i++){
          char hex[3];
          string2hexString(hex, &deciphered_3[i], 2);
          std::string printableHex(hex, 2);
          if(i < 200){
            log_recovered_3 << printableHex << " ";
          }  
        }
        
      
        log_recovered_3.close();

        delete[] deciphered_3;

        
        sensor_msgs::PointCloud2 cloud_msg_copy3;
        cloud_msg_copy3 = cloud_msg_list;

        // update pointcloud2 ros message
        for(int i = 0; i < size_cloud2; i++){
          cloud_msg_copy3.data[i] = deciphered_4[i];
        }

        lidar_recovered.publish(cloud_msg_copy3);

        delete[] deciphered_4;



      //}

      //else{
      //  std::cout << "not received from listener" << std::endl;
      //}

     
    //}
    //else
    //{
    //  std::cout << "rosbag not received" << std::endl;
    //}
    

      
    //log_recovered.close();

    ros::spinOnce();
    //loop_rate.sleep();


    
  }
  


  return 0;
}