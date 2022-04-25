cmake_minimum_required(VERSION 3.10)
project(spark_backend VERSION 0.0.1)
set(CMAKE_CXX_STANDARD 17)

# TODO: turn off the verbose cmake outputs to the console
set(CMAKE_VERBOSE_MAKEFILE off)

include_directories(${CMAKE_CURRENT_LIST_DIR}/common/)
link_directories(${CMAKE_CURRENT_LIST_DIR}/lib/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/modules/)
include_directories(${PROJECT_SOURCE_DIR}/ros/include/) 
include_directories(${CMAKE_CURRENT_LIST_DIR}/ros/include/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/common/gripper/)

# TODO: Will GLOB search find all the files and folders in a directory? What if there are files that are unintentionally
# included in the directory? Is there a way to only get the folder names? 
file(GLOB lib_dirs ${CMAKE_CURRENT_LIST_DIR}/lib/*)
message("--------------------------\n")
foreach(dir ${lib_dirs})
    message(${dir})
endforeach()
message("--------------------------\n")
link_directories(${lib_dirs})

# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/modules/communication/ros_comm)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/modules/)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/apps/)

# TODO: this subdirectory is for testing only. 
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/test/)

# Packages that have little change of polluting the naming space, 
# and used commonly for all classes
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})



