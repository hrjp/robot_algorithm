cmake_minimum_required(VERSION 3.10.2)
project(a_star)
 
set(CMAKE_CXX_STANDARD 11)
 

set(LIB_INCLUDE_DIRS "../include")
include_directories(${LIB_INCLUDE_DIRS})

add_executable(a_star main.cpp)

 
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(a_star ${PYTHON_LIBRARIES})
