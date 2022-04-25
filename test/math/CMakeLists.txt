set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(euler_quat euler_quat.cpp)
target_link_libraries(euler_quat MathUtils)