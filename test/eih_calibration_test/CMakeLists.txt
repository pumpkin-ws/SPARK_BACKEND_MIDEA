set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

include_directories(${PROJECT_SOURCE_DIR}/modules/vision/calibration/include/)
include_directories(${PROJECT_SOURCE_DIR}/modules/math/include/)
# Halcon
include_directories(/opt/halcon/include/halconcpp)
include_directories(/opt/halcon/include)
link_directories(/opt/halcon/lib/x64-linux)

# include the catkin package
find_package (
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(calib-verify calib_verify.cpp)
target_link_libraries(calib-verify Calibration halcon halconcpp)

add_executable(calib-service calib-service.cpp)
target_link_libraries(calib-service ${catkin_LIBRARIES})

add_executable(calib-trans eih-transformation.cpp)
target_link_libraries(calib-trans MathUtils)

add_executable(pose_transform pose_transform.cpp)
target_link_libraries(pose_transform MathUtils)

