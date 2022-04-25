set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

find_package(OpenCV 4 REQUIRED)

find_package(yaml-cpp REQUIRED)

add_executable(yaml_read yaml_read.cpp)
target_link_libraries(yaml_read ${OpenCV_LIBS})

add_executable(yaml_write yaml_write.cpp)
target_link_libraries(yaml_write ${OpenCV_LIBS})

add_executable(yaml_cpp_read yaml_cpp_read.cpp)
target_link_libraries(yaml_cpp_read yaml-cpp)