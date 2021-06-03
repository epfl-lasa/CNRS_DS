cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(CNRS_DS LANGUAGES CXX VERSION 1.0.0)

find_package(Eigen3 REQUIRED eigen3)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(CNRS_DS src/main.cpp src/nominal_DS.cpp src/modulated_DS.cpp src/calculate_alpha.cpp)
