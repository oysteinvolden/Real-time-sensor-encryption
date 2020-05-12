cmake_minimum_required(VERSION 2.8.3)
project(crypto_pipeline)

## Find catkin and any catkin packages
## find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    image_transport
)

## Declare ROS messages and services
##add_message_files(FILES Num.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
##generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
## catkin_package()
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    rospy
    std_msgs
    image_transport
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


## uncomment executable to get correct executable below 


## HC-128-HMAC point cloud
##add_executable(talker src/pointcloud/HC-128-HMAC/talker_cloud_4.cpp src/pointcloud/HC-128-HMAC/hc128.cpp src/pointcloud/HC-128-HMAC/encoder.cpp src/pointcloud/HC-128-HMAC/hmac.cpp src/pointcloud/HC-128-HMAC/aes_cfb.cpp src/pointcloud/HC-128-HMAC/sha-256.cpp)

## HC-128-HMAC video
#add_executable(talker src/video/HC-128-HMAC/talker_cloud_4.cpp src/video/HC-128-HMAC/hc128.cpp src/video/HC-128-HMAC/encoder.cpp src/video/HC-128-HMAC/hmac.cpp src/video/HC-128-HMAC/aes_cfb.cpp src/video/HC-128-HMAC/sha-256.cpp)

## HC-128 for point cloud
#add_executable(talker src/pointcloud/HC-128/talker_cloud_4.cpp src/pointcloud/HC-128/hc128.cpp src/pointcloud/HC-128/encoder.cpp src/pointcloud/HC-128/aes_cfb.cpp)

## HC-128 for video 
#add_executable(talker src/video/HC-128/talker_cloud_4.cpp src/video/HC-128/hc128.cpp src/video/HC-128/encoder.cpp src/video/HC-128/aes_cfb.cpp)

## aes_cfb for video
#add_executable(talker src/video/aes_cfb/talker.cpp src/video/aes_cfb/aes_cfb.cpp)

## aes_cfb for point cloud
#add_executable(talker src/pointcloud/aes_cfb/talker_cloud_4.cpp src/pointcloud/aes_cfb/aes_cfb.cpp)

## Sosemanuk for point cloud
#add_executable(talker src/pointcloud/Sosemanuk/talker.cpp src/pointcloud/Sosemanuk/sosemanuk.cpp src/pointcloud/Sosemanuk/serpent.cpp src/pointcloud/Sosemanuk/encoder.cpp)

## Sosemanuk for video
add_executable(talker src/video/Sosemanuk/talker.cpp src/video/Sosemanuk/sosemanuk.cpp src/video/Sosemanuk/serpent.cpp src/video/Sosemanuk/encoder.cpp)

## no encryption point cloud
#add_executable(talker src/pointcloud/no_encryption/talker.cpp)

## no encryption video
#add_executable(talker src/video/no_encryption/talker.cpp)

##target_link_libraries(talker ${catkin_LIBRARIES})

target_link_libraries(talker
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(talker crypto_pipeline_generate_messages_cpp)


## HC-128-HMAC point cloud
##add_executable(listener src/pointcloud/HC-128-HMAC/listener_cloud_4.cpp src/pointcloud/HC-128-HMAC/hc128.cpp src/pointcloud/HC-128-HMAC/encoder.cpp src/pointcloud/HC-128-HMAC/hmac.cpp src/pointcloud/HC-128-HMAC/aes_cfb.cpp src/pointcloud/HC-128-HMAC/sha-256.cpp)

## HC-128-HMAC video
#add_executable(listener src/video/HC-128-HMAC/listener_cloud_4.cpp src/video/HC-128-HMAC/hc128.cpp src/video/HC-128-HMAC/encoder.cpp src/video/HC-128-HMAC/hmac.cpp src/video/HC-128-HMAC/aes_cfb.cpp src/video/HC-128-HMAC/sha-256.cpp)

## HC-128 for point cloud
#add_executable(listener src/pointcloud/HC-128/listener_cloud_4.cpp src/pointcloud/HC-128/hc128.cpp src/pointcloud/HC-128/encoder.cpp src/pointcloud/HC-128/aes_cfb.cpp)

## HC-128 for video
#add_executable(listener src/video/HC-128/listener_cloud_4.cpp src/video/HC-128/hc128.cpp src/video/HC-128/encoder.cpp src/video/HC-128/aes_cfb.cpp)

## aes_cfb for video
##add_executable(listener src/video/aes_cfb/listener.cpp src/video/aes_cfb/aes_cfb.cpp)

## aes_cfb for point cloud
#add_executable(listener src/pointcloud/aes_cfb/listener_cloud_4.cpp src/pointcloud/aes_cfb/aes_cfb.cpp)

## Sosemanuk for point cloud
#add_executable(listener src/pointcloud/Sosemanuk/listener.cpp src/pointcloud/Sosemanuk/sosemanuk.cpp src/pointcloud/Sosemanuk/serpent.cpp src/pointcloud/Sosemanuk/encoder.cpp)

## Sosemanuk for video
add_executable(listener src/video/Sosemanuk/listener.cpp src/video/Sosemanuk/sosemanuk.cpp src/video/Sosemanuk/serpent.cpp src/video/Sosemanuk/encoder.cpp)


## no encryption point cloud
#add_executable(listener src/pointcloud/no_encryption/listener.cpp)

## no encryption video
#add_executable(listener src/video/no_encryption/listener.cpp) 

##target_link_libraries(listener ${catkin_LIBRARIES})

target_link_libraries(listener
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(listener crypto_pipeline_generate_messages_cpp)
