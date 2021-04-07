# version 3.11 or later of CMake needed later for installing GoogleTest
# so let's require it now.
cmake_minimum_required(VERSION 3.10-3.18)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(
    "jprsim"
    VERSION 1.0
    DESCRIPTION "F-16 Flight Simulator - Grad School Thesis Project")

add_executable(
    jprsim 
    "${PROJECT_SOURCE_DIR}/source/jprsim.cpp" 
    "${PROJECT_SOURCE_DIR}/source/socketUDP.cpp"
)

target_include_directories(
    jprsim 
    PUBLIC 
    "${PROJECT_SOURCE_DIR}/headers" 
    "${PROJECT_SOURCE_DIR}/headers/aircraft/F16"
    "${PROJECT_SOURCE_DIR}/source"
)
