set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    image_transport
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cam_subscriber cam_subscriber.cpp)

target_link_libraries(
    cam_subscriber
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)