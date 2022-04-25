set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(FE_simulator front_end_simulator.cpp)

target_link_libraries(
    FE_simulator
    ${catkin_LIBRARIES}
    pthread
)