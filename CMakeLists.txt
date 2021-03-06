macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()



cmake_minimum_required(VERSION 2.8.9)
project(ESKF)

set(CMAKE_CXX_FLAGS "-O0 -W -Wall -std=c++11 -g -fpic -shared -Wl,-fvisibility=hidden ")

#for debugging it is nice to use ros, though this can be deleted if one is to remove the includes in the parsing files.
#[[
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
)
]]


file(GLOB INCLUDES "include/*.h" "include/fast-cpp-csv-parser/*.h" "eigen/*.h" "eigen/src/*/*.h")

#include_directories(./include eigen eigen/src/*/* include/fast-cpp-csv-parser ${catkin_INCLUDE_DIRS})

include_directories(./include eigen eigen/src/*/* include/fast-cpp-csv-parser ${catkin_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp" "include/*.h" "eigen/*.h" "eigen/src/*/*.h")

add_library(ESKF SHARED ${SOURCES})

target_link_libraries(ESKF)

print_all_variables()

install(TARGETS ESKF
    LIBRARY DESTINATION .
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
