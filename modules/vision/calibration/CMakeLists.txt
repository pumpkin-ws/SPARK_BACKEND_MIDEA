set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/vision/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/vision/)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Halcon
include_directories(/opt/halcon/include/halconcpp)
include_directories(/opt/halcon/include)
link_directories(/opt/halcon/lib/x64-linux)

# Yaml
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_CMAKE_DIR})
include_directories(${YAML_CPP_CMAKE_DIR}/include/)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)
file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(Calibration STATIC ${sources})
target_link_libraries(
    Calibration 
    ${OpenCV_LIBS}
    halcon
    halconcpp)
target_link_libraries(Calibration yaml-cpp)



