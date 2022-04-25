set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(manager_client manager_client.cpp)
target_link_libraries(
    manager_client 
    ${catkin_LIBRARIES}
    pthread
    MathUtils
)