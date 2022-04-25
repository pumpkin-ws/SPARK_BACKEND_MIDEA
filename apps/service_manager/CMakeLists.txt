set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/service_manager)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/service_manager)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/service_manager)
set(CMAKE_CXX_FLAGS -fPIC)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_CMAKE_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)

find_package(Boost REQUIRED filesystem)
include_directories(${Boost_INCLUDE_DIRS})


file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)
foreach(f ${sources})
    message(${f})
endforeach()

add_library(ServiceManager SHARED ${sources}) # can only build static library because of yaml
target_link_libraries(ServiceManager 
                      ${catkin_LIBRARIES}
                      pthread
                      yaml-cpp
                      ${Boost_LIBRARIES}
                      plcIO
                      FileSystem)

add_executable(ManagerNode spark_manager_node.cpp)
target_link_libraries(
    ManagerNode 
    ServiceManager
    MathUtils
    yaml-cpp
)
