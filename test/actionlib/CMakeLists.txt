set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

set(CMAKE_CXX_STANDARD 17)
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(server test_server.cpp)
target_link_libraries(server ${catkin_LIBRARIES} pthread)

add_executable(client test_client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} pthread)