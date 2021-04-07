# version 3.11 or later of CMake needed later for installing GoogleTest
# so let's require it now.
cmake_minimum_required(VERSION 3.11-3.18)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(
    "jprsim"
    VERSION 1.0
    DESCRIPTION "F-16 Flight Simulator - Grad School Thesis Project")

target_include_directories(headers PUBLIC "${PROJECT_BINARY_DIR}/headers")
target_include_directories(source PUBLIC "${PROJECT_BINARY_DIR}/source")