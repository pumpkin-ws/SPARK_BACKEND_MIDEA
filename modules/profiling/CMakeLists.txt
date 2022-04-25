set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/profiling/)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)
file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

add_library(
    profile 
    SHARED 
    ${sources})