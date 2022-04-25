set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)

# include the aubo dependents into this project
# CMAKE_CURRENT_LIST_DIR is the directory where the current CMakeLists.txt is.
set(AUBO_DIR ${PROJECT_SOURCE_DIR}/3rdparty/aubo_sdk)
# log4cplus directory and aubosdk directory
include_directories(${AUBO_DIR}/log4cplus/linux_x64/inc/log4cplus)
include_directories(${AUBO_DIR}/robotSDK/inc)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(robot_client robotclient.cpp)
target_link_libraries(robot_client ${catkin_LIBRARIES})

add_executable(robot_test robot_test.cpp)
target_link_libraries(robot_test AuboMover)

add_executable(movel-test movel_test.cpp)
target_link_libraries(movel-test AuboMover)

add_executable(move_line move_line.cpp)
target_link_libraries(move_line ${catkin_LIBRARIES})

add_executable(move_track move_track.cpp)
target_link_libraries(move_track ${catkin_LIBRARIES})

add_executable(get_io get_io.cpp)
target_link_libraries(get_io ${catkin_LIBRARIES})
