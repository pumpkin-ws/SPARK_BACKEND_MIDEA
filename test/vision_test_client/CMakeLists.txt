set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vision_test_client vision_test_client.cpp)
target_link_libraries(vision_test_client ${catkin_LIBRARIES})

add_executable(generate_template generate_template.cpp)
target_link_libraries(generate_template ${catkin_LIBRARIES})

add_executable(track_target track_target.cpp)
target_link_libraries(track_target ${catkin_LIBRARIES})