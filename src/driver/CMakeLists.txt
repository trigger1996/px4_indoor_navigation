cmake_minimum_required(VERSION 2.8.3)

find_package(OpenCV REQUIRED

)

include_directories(
#include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_cam_driver
    src/driver/camera_driver_main.cpp
)

target_link_libraries(${PROJECT_NAME}_cam_driver
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
