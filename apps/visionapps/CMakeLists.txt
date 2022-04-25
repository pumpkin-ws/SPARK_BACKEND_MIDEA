set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/vision/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/vision/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/vision/)

# ROS
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    cv_bridge
    image_transport
    geometry_msgs
)
include_directories(${catkin_INCLUDE_DIRS})


# OPENCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL 
find_package(PCL 1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Halcon
include_directories(/opt/halcon/include/halconcpp)
include_directories(/opt/halcon/include)
link_directories(/opt/halcon/lib/x64-linux)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)

file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

add_library(VisionServices STATIC ${sources})
target_link_libraries(
    VisionServices
    Calibration
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES} 
    VisionUtils
    VisionTracker
    MathUtils
)

add_executable(VisionNode vision_node.cpp)
target_link_libraries(
    VisionNode 
    ${catkin_LIBRARIES} 
    pthread
    VisionServices
    FileSystem)