set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/media)

find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    cv_bridge
    image_transport
)

include_directories(${catkin_INCLUDE_DIRS})

find_package(PCL 1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(
    eihstream
    eihstream.cpp
)

target_link_libraries(
    eihstream 
    ${OpenCV_LIBS} 
    ${catkin_LIBRARIES}
    rsDriver
)
