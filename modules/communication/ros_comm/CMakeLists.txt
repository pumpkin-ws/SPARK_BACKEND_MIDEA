# set the build file to a directory more accessible to the user, the build directory will be deleted after every update
cmake_minimum_required(VERSION 3.10)
set(CATKIN_DEVEL_PREFIX ${PROJECT_SOURCE_DIR}/ros/)
find_package(
    catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    actionlib
    actionlib_msgs
    message_runtime
    sensor_msgs
    geometry_msgs
)

# Declare the message files to be built
add_message_files(
    DIRECTORY
    msg
    FILES
    AuboInfo.msg
    OmronPLCInfo.msg
)

# Declare the service files to be built
add_service_files(
    DIRECTORY
    srv
    FILES
    VisionClientServer.srv
    RobotClientServer.srv
)

# Declare the action files to be built
add_action_files(
    DIRECTORY
    action
    FILES
    RobotService.action
    VisionService.action
    ServiceManager.action
)
message("Generating Aubo messages!")
# Generate messages
generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
    sensor_msgs
    geometry_msgs
)