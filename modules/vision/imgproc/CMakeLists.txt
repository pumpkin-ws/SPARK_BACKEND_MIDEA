set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/vision/)

# OpenCV
find_package(OpenCV 4 REQUIRED)

# TODO: Make the inclusion of Halcon not directly through directories
include_directories(/opt/halcon/include/halconcpp
                    /opt/halcon/include)
link_directories(/opt/halcon/lib/x64-linux)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)
file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

add_library(ImageProc SHARED ${sources})
target_link_libraries(ImageProc
                        ${OpenCV_LIBS}
                        halcon
                        halconcpp)