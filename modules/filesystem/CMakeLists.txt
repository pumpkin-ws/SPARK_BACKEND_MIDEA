set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/filesystem/)

find_package(Boost REQUIRED filesystem)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)

file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)
add_library(FileSystem SHARED ${sources})
target_link_libraries(FileSystem ${Boost_LIBRARIES})