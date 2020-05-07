# Real-time encryption of sensors in autonomous systems

## Overview
This repo contains source code and instructions to interface C++ implemetations of cryptological algorithms for different sensor data such as images / video stream, point cloud and control signals. By following the instructions, you should be able to create an efficient pipeline to transfer different types of sensor data securely across machines. I.e, sensor data is encrypted during transfer and only decrypted at end-points. Authentication algorithms from the toolbox is also included to ensure that data is not changed during transfer.

We use Robot Operating System (ROS) as a software platform to handle sensor interfacing and low-level communication between nodes (either locally on one single machine or across multiple machines). This simplfies the task of applying the cryptological toolbox of algorithms for different sensor data significantly. Fortunately, ROS also offers point cloud libraries to interface and visualize lidar data. In addition, we use OpenCV to interface image data for encryption/decryption operations. I.e. turning high-level images into serialized data to fit input buffers and vice versa, deserialize the data from output buffers into high-level images. 
 
This repo is in fact a ROS package which can easily be integrated into a ROS environment applied by new users. It is tested with Ubuntu 18.04 LTS and ROS melodic, both on x86 architecture (standard laptop) and arm-based 64-bit architecture (Nvidia Jetson Xavier). In the src folder, each application folder is listed and under each application folder, each cryptological method in use is listed. In CMakeLists.txt, one can easily comment / uncomment executives representing the different cryptological methods applied to different sensor data (video, pointcloud or control signals). Remember to only include one pair of executive at the time ("talker" - the ROS node to send data and "listener" the ROS node to receive data). For simplicity, all internal crypto libraries neccessary for each application is stored locally. This may be changed later. 

## Installation

### Dependencies
This software is built on ROS, which needs to be installed first. If you use a standard PC with x86 architecture, follow the instructions here: http://wiki.ros.org/melodic/Installation/Ubuntu. Full-desktop version is recommended if disk-space is not critical.

For arm-based Nvidia Jetson Xavier, clone the repository: https://github.com/jetsonhacks/installROSXavier, cd into it and run: ./installROS.sh -p ros-melodic-desktop-full. This repo also provide a quick solution to setup a catkin workspace by running the command: ./setupCatkinWorkspace.sh. More instructions on how to setup a Nvidia Jetson Xavier from scratch is included in the documentation folder. 

In addition, to be able to run the image/video encryption examples, this ROS package depends on the following software:

- [OpenCV](http://opencv.org/) (computer vision library)

For standard laptops, instructions from here is recommended: https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/
For Jetson Xavier, it is recommended to clone this repo: https://github.com/AastaNV/JEP, cd into script and run: ./install_opencv4.1.1_Jetson.sh. 
NB: There has been some issues when combining ROS melodic and OpenCV 4.x.x, so it may be more safe to install OpenCV 3.4.x for instance. We installed 3.4.3 by simply changing 4.1.1 with 3.4.3 in the sh file.

Now, create a catkin workspace and include our ROS package as well as ROS package for bridging opencv and ROS (vision_opencv):

    cd catkin_ws/src
    git clone https://github.com/oysteinvolden/Real-time-sensor-encryption.git
    git clone https://github.com/ros-perception/vision_opencv.git 
    cd ..
    catkin_make -DCMAKE_BUILD_TYPE=Release

Building in release mode makes sure you maximize performance. 

## Basic Usage
Run the publisher and the subscriber:
    - Open a terminal and type: roscore
	- Open a second terminal and:
		- cd ~/catkin_ws
		- source devel/setup.bash
		- rosrun crypto_pipeline talker
	- Open a third terminal and:
		- cd ~/catkin_ws
		- source devel/setup.bash
		- rosrun crypto_pipeline listener


Additional handy ROS tools in the terminal:
	
    - Check topics pulished: rostopic list -v
    - Check content of topics published: rostopic echo \topic_name
    - Check frequency: rostopic hz \topic_name
    - Visualize image topic: rqt_image_view
    - Visualize point cloud:
        - rviz
        - change frame to "os1_lidar"





TODO: 
- refer / credit crypto toolbox



**Authors: [Oystein Volden](https://www.ntnu.no/ansatte/oystv), oystein.volden@ntnu.no and [Petter Solnoer](https://www.ntnu.no/ansatte/petter.solnor), petter.solnor@ntnu.no**