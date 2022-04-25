set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

add_executable(file_system_test file_system_test.cpp)

target_link_libraries(
    file_system_test 
    FileSystem)