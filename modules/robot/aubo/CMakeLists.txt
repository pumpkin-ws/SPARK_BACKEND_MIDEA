set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/robot/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/robot/)

# include the aubo dependents into this project
# CMAKE_CURRENT_LIST_DIR is the directory where the current CMakeLists.txt is.
set(AUBO_DIR ${PROJECT_SOURCE_DIR}/3rdparty/aubo_sdk)
# log4cplus directory and aubosdk directory
include_directories(${AUBO_DIR}/log4cplus/linux_x64/inc/log4cplus)
include_directories(${AUBO_DIR}/robotSDK/inc)

file(GLOB log4plus_LIBS "${AUBO_DIR}/log4cplus/linux_x64/lib/*" ABSOLUTE) # ? what is the ABSOLUTE keyword used for?
file(GLOB auboSDK_LIBS "${AUBO_DIR}/robotSDK/lib/linux_x64/*" ABSOLUTE)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include/)
file(GLOB sources ${CMAKE_CURRENT_LIST_DIR}/source/*.cpp)

# foreach(s ${sources})
#     message("------------------\n")
#     message(${s}\n)
#     message("------------------\n")
# endforeach()

add_library(AuboMover SHARED ${sources})
target_link_libraries(AuboMover 
                        ${log4plus_LIBS}
                        ${auboSDK_LIBS} redis++)

