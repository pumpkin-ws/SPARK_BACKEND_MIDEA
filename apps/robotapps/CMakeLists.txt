set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/robot/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/robot/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

# include the aubo dependents into this project
# CMAKE_CURRENT_LIST_DIR is the directory where the current CMakeLists.txt is.
set(AUBO_DIR ${PROJECT_SOURCE_DIR}/3rdparty/aubo_sdk/)
# log4cplus directory and aubosdk directory
include_directories(${AUBO_DIR}/log4cplus/linux_x64/inc/log4cplus)
include_directories(${AUBO_DIR}/robotSDK/inc)

file(GLOB log4plus_LIBS "${AUBO_DIR}/log4cplus/linux_x64/lib/*" ABSOLUTE) # ? what is the ABSOLUTE keyword used for?
file(GLOB auboSDK_LIBS "${AUBO_DIR}/robotSDK/lib/linux_x64/*" ABSOLUTE)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)

file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

add_library(RobotServices ${sources})
target_link_libraries(RobotServices AuboMover ${catkin_LIBRARIES}) 

add_executable(RobotNode robot_node.cpp)
target_link_libraries(RobotNode 
                      ${catkin_LIBRARIES} 
                      pthread
                      RobotServices
                      AuboMover)


