cmake_minimum_required(VERSION 3.10.2)
project(a_star)
 
set(CMAKE_CXX_STANDARD 11)
 

set(LIB_INCLUDE_DIRS "../include")
include_directories(${LIB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
