set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(move_robot move_robot.cpp)
target_link_libraries(move_robot ${catkin_LIBRARIES})
target_link_libraries(move_robot pthread)

add_executable(move_joint_angle move_joint_angle.cpp)
target_link_libraries(move_joint_angle ${catkin_LIBRARIES})