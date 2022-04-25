set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/message/)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

set(AUBO_DIR ${PROJECT_SOURCE_DIR}/3rdparty/aubo_sdk/)
include_directories(${AUBO_DIR}/log4cplus/linux_x64/inc/log4cplus)
include_directories(${AUBO_DIR}/robotSDK/inc)

add_executable(aubo_publisher aubo_publisher.cpp)
target_link_libraries(
    aubo_publisher 
    ${catkin_LIBRARIES}
    AuboMover)

add_executable(aubo_wp_publisher aubo_wp_publisher.cpp)
target_link_libraries(
    aubo_wp_publisher
    ${catkin_LIBRARIES}
    AuboMover
)

add_executable(aubo_io_publisher aubo_io_publisher.cpp)
target_link_libraries(
    aubo_io_publisher
    ${catkin_LIBRARIES}
    AuboMover
)

add_executable(omron_plc_publisher omron_plc_publisher.cpp)
target_link_libraries(
    omron_plc_publisher
    ${catkin_LIBRARIES}
    plcIO
)