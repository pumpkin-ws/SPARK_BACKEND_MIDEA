set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/math/)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)

find_package(Eigen3 REQUIRED)

file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

# message("-----------message from math----------------")

# foreach(s ${sources})
#     message(${s})
# endforeach()

# message("-----------message from math----------------")
add_library(MathUtils ${sources})
