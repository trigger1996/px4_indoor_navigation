cmake_minimum_required(VERSION 2.8.3)

set( CMAKE_BUILD_TYPE "Release" )

find_package(catkin REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED)
find_package(cv_bridge)
find_package(image_transport)
find_package(dynamic_reconfigure)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
#find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)
find_package(  Boost REQUIRED COMPONENTS filesystem )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

include_directories(
    src/visual_obstacle_map/include
    src/visual_obstacle_map/cfg/cpp
    )



add_library(pcl_helper 
	   src/visual_obstacle_map/src/pcl_helper.cpp
    )


target_link_libraries(pcl_helper
         ${PCL_LIBRARIES}
    )


add_executable(dense_reconstruction
	src/visual_obstacle_map/src/dense_reconstruction.cpp
    )

message("2333333333333333333333333333333333333333333333"  ${image_transport_LIBRARIES})

target_link_libraries(dense_reconstruction
         ${OpenCV_LIBS}
         ${catkin_LIBRARIES}
         "-lpopt -lboost_system"
          pcl_helper
    )

add_executable(match_pointcloud_and_slam_pose
    src/visual_obstacle_map/src/match_pointcloud_and_slam_pose.cpp
    )

target_link_libraries(match_pointcloud_and_slam_pose 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
     "-lpopt -lboost_system"
    pcl_helper)

add_executable(match_pointcloud_and_slam_pose2
    src/visual_obstacle_map/src/match_pointcloud_and_slam_pose2.cpp
    )

target_link_libraries(match_pointcloud_and_slam_pose2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
     "-lpopt -lboost_system"
    pcl_helper)

#rosbuild_add_executable(rgbd_camera_to_pointcloud
#    src/visual_obstacle_map/src/rgbd_camera_to_pointcloud.cpp
#    )
#
#target_link_libraries(rgbd_camera_to_pointcloud
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#    ${catkin_LIBRARIES}
#     "-lpopt -lboost_system"
#    pcl_helper
#    )


