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

### ENCRYPTION ###

## HC-128 for point cloud
#add_executable(talker src/pointcloud/HC-128/talker_cloud_4.cpp src/pointcloud/HC-128/hc128.cpp src/pointcloud/HC-128/encoder.cpp src/pointcloud/HC-128/aes_cfb.cpp)

## HC-128 for video 
#add_executable(talker src/video/HC-128/talker_2.cpp src/video/HC-128/hc128.cpp src/video/HC-128/encoder.cpp src/video/HC-128/aes_cfb.cpp)

## aes_cfb for video
#add_executable(talker src/video/aes_cfb/talker_2.cpp src/video/aes_cfb/aes_cfb.cpp)

## aes_cfb for point cloud
#add_executable(talker src/pointcloud/aes_cfb/talker_cloud_4.cpp src/pointcloud/aes_cfb/aes_cfb.cpp)

## Sosemanuk for point cloud
#add_executable(talker src/pointcloud/Sosemanuk/talker.cpp src/pointcloud/Sosemanuk/sosemanuk.cpp src/pointcloud/Sosemanuk/serpent.cpp src/pointcloud/Sosemanuk/encoder.cpp)

## Sosemanuk for video
#add_executable(talker src/video/Sosemanuk/talker_2.cpp src/video/Sosemanuk/sosemanuk.cpp src/video/Sosemanuk/serpent.cpp src/video/Sosemanuk/encoder.cpp)

## Rabbit for pointcloud
#add_executable(talker src/pointcloud/Rabbit/talker.cpp src/pointcloud/Rabbit/rabbit.cpp src/pointcloud/Rabbit/encoder.cpp)

## Rabbit for video
#add_executable(talker src/video/Rabbit/talker.cpp src/video/Rabbit/rabbit.cpp src/video/Rabbit/encoder.cpp)

## ChaCha20 for video
#add_executable(talker src/video/ChaCha20/talker.cpp src/video/ChaCha20/chacha.cpp src/video/ChaCha20/encoder.cpp)

## ChaCha20 for pointcloud
#add_executable(talker src/pointcloud/ChaCha20/talker.cpp src/pointcloud/ChaCha20/chacha.cpp src/pointcloud/ChaCha20/encoder.cpp)



### OTHER STUFF ###

## no encryption point cloud
#add_executable(talker src/pointcloud/no_encryption/talker.cpp)

## no encryption video
#add_executable(talker src/video/no_encryption/talker_2.cpp)

## video compression + AES encryption
add_executable(talker src/video/video_compression_encryption/talker.cpp src/video/video_compression_encryption/aes_cfb.cpp)



### AUTHENTICATED ENCRYPTION ##

## HC-128-HMAC point cloud
#add_executable(talker src/pointcloud/HC-128-HMAC/talker_cloud_4.cpp src/pointcloud/HC-128-HMAC/hc128.cpp src/pointcloud/HC-128-HMAC/encoder.cpp src/pointcloud/HC-128-HMAC/hmac.cpp src/pointcloud/HC-128-HMAC/aes_cfb.cpp src/pointcloud/HC-128-HMAC/sha-256.cpp)

## HC-128-HMAC video
#add_executable(talker src/video/HC-128-HMAC/talker_2.cpp src/video/HC-128-HMAC/hc128.cpp src/video/HC-128-HMAC/encoder.cpp src/video/HC-128-HMAC/hmac.cpp src/video/HC-128-HMAC/aes_cfb.cpp src/video/HC-128-HMAC/sha-256.cpp)

## Rabbit-HMAC video
#add_executable(talker src/video/Rabbit-HMAC/talker.cpp src/video/Rabbit-HMAC/rabbit.cpp src/video/Rabbit-HMAC/encoder.cpp src/video/Rabbit-HMAC/hmac.cpp src/video/Rabbit-HMAC/sha-256.cpp)

## Rabbit-HMAC pointcloud
#add_executable(talker src/pointcloud/Rabbit-HMAC/talker.cpp src/pointcloud/Rabbit-HMAC/rabbit.cpp src/pointcloud/Rabbit-HMAC/encoder.cpp src/pointcloud/Rabbit-HMAC/hmac.cpp src/pointcloud/Rabbit-HMAC/sha-256.cpp)

# AEGIS video
#add_executable(talker src/video/AEGIS/talker.cpp src/video/AEGIS/aegis_128.cpp src/video/AEGIS/encoder.cpp)

# AEGIS pointcloud
#add_executable(talker src/pointcloud/AEGIS/talker.cpp src/pointcloud/AEGIS/aegis_128.cpp src/pointcloud/AEGIS/encoder.cpp)

# AEGIS video HW acceleration
add_executable(talker src/video/AEGIS_ARM/talker.cpp src/video/AEGIS_ARM/aegis_128.cpp src/video/AEGIS_ARM/encoder.cpp)


### COMPRESSION + AUTHENTICATED ENCRYPTION ##


##target_link_libraries(talker ${catkin_LIBRARIES})

target_link_libraries(talker
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(talker crypto_pipeline_generate_messages_cpp)

### ENCRYPTION ###

## HC-128 for point cloud
#add_executable(listener src/pointcloud/HC-128/listener_cloud_4.cpp src/pointcloud/HC-128/hc128.cpp src/pointcloud/HC-128/encoder.cpp src/pointcloud/HC-128/aes_cfb.cpp)

## HC-128 for video
#add_executable(listener src/video/HC-128/listener_2.cpp src/video/HC-128/hc128.cpp src/video/HC-128/encoder.cpp src/video/HC-128/aes_cfb.cpp)

## aes_cfb for video
#add_executable(listener src/video/aes_cfb/listener_2.cpp src/video/aes_cfb/aes_cfb.cpp)

## aes_cfb for point cloud
#add_executable(listener src/pointcloud/aes_cfb/listener_cloud_4.cpp src/pointcloud/aes_cfb/aes_cfb.cpp)

## Sosemanuk for point cloud
#add_executable(listener src/pointcloud/Sosemanuk/listener.cpp src/pointcloud/Sosemanuk/sosemanuk.cpp src/pointcloud/Sosemanuk/serpent.cpp src/pointcloud/Sosemanuk/encoder.cpp)

## Sosemanuk for video
#add_executable(listener src/video/Sosemanuk/listener_2.cpp src/video/Sosemanuk/sosemanuk.cpp src/video/Sosemanuk/serpent.cpp src/video/Sosemanuk/encoder.cpp)

## Rabbit for pointcloud
#add_executable(listener src/pointcloud/Rabbit/listener.cpp src/pointcloud/Rabbit/rabbit.cpp src/pointcloud/Rabbit/encoder.cpp)

## Rabbit for video
#add_executable(listener src/video/Rabbit/listener.cpp src/video/Rabbit/rabbit.cpp src/video/Rabbit/encoder.cpp)

## ChaCha20 for video
#add_executable(listener src/video/ChaCha20/listener.cpp src/video/ChaCha20/chacha.cpp src/video/ChaCha20/encoder.cpp)

## ChaCha20 for pointcloud
#add_executable(listener src/pointcloud/ChaCha20/listener.cpp src/pointcloud/ChaCha20/chacha.cpp src/pointcloud/ChaCha20/encoder.cpp)



### OTHER STUFF ###

## no encryption point cloud
#add_executable(listener src/pointcloud/no_encryption/listener.cpp)

## no encryption video
#add_executable(listener src/video/no_encryption/listener_2.cpp)

## video compression + AES encryption
add_executable(listener src/video/video_compression_encryption/listener.cpp src/video/video_compression_encryption/aes_cfb.cpp)



### AUTHENTICATED ENCRYPTION ###

## HC-128-HMAC point cloud
#add_executable(listener src/pointcloud/HC-128-HMAC/listener_cloud_4.cpp src/pointcloud/HC-128-HMAC/hc128.cpp src/pointcloud/HC-128-HMAC/encoder.cpp src/pointcloud/HC-128-HMAC/hmac.cpp src/pointcloud/HC-128-HMAC/aes_cfb.cpp src/pointcloud/HC-128-HMAC/sha-256.cpp)

# HC-128-HMAC video
#add_executable(listener src/video/HC-128-HMAC/listener_2.cpp src/video/HC-128-HMAC/hc128.cpp src/video/HC-128-HMAC/encoder.cpp src/video/HC-128-HMAC/hmac.cpp src/video/HC-128-HMAC/aes_cfb.cpp src/video/HC-128-HMAC/sha-256.cpp)

# Rabbit-HMAC video
#add_executable(listener src/video/Rabbit-HMAC/listener.cpp src/video/Rabbit-HMAC/rabbit.cpp src/video/Rabbit-HMAC/encoder.cpp src/video/Rabbit-HMAC/hmac.cpp src/video/Rabbit-HMAC/sha-256.cpp)

# Rabbit-HMAC pointcloud
#add_executable(listener src/pointcloud/Rabbit-HMAC/listener.cpp src/pointcloud/Rabbit-HMAC/rabbit.cpp src/pointcloud/Rabbit-HMAC/encoder.cpp src/pointcloud/Rabbit-HMAC/hmac.cpp src/pointcloud/Rabbit-HMAC/sha-256.cpp)

# AEGIS video
#add_executable(listener src/video/AEGIS/listener.cpp src/video/AEGIS/aegis_128.cpp src/video/AEGIS/encoder.cpp)

# AEGIS pointcloud
#add_executable(listener src/pointcloud/AEGIS/listener.cpp src/pointcloud/AEGIS/aegis_128.cpp src/pointcloud/AEGIS/encoder.cpp)

# AEGIS video HW acceleration
add_executable(listener src/video/AEGIS_ARM/listener.cpp src/video/AEGIS_ARM/aegis_128.cpp src/video/AEGIS_ARM/encoder.cpp)



##target_link_libraries(listener ${catkin_LIBRARIES})

target_link_libraries(listener
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(listener crypto_pipeline_generate_messages_cpp)


### for ARM - HW acceleration ###

#add_compile_options(-march=armv8-a+crypto)
#add_compile_options(-march=westmere)

