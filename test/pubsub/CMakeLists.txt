set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(message_publisher custom_message_publisher.cpp)
target_link_libraries(message_publisher ${catkin_LIBRARIES})

add_executable(message_subscriber custom_message_subscriber.cpp)
target_link_libraries(message_subscriber ${catkin_LIBRARIES})

add_executable(aubo_info_subscriber aubo_info_subscriber.cpp)
target_link_libraries(aubo_info_subscriber ${catkin_LIBRARIES})