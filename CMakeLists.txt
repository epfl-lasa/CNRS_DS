cmake_minimum_required(VERSION 3.0.2)

project(CNRS_DS)

find_package(Eigen3 REQUIRED eigen3) 

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(CNRS_DS src/main.cpp src/nominal_DS.cpp src/modulated_DS.cpp src/calculate_alpha.cpp)

target_link_libraries(CNRS_DS ${Eigen3_LIBRARIES} "stdc++fs")

set_property(TARGET CNRS_DS PROPERTY CXX_STANDARD 17)
set_property(TARGET CNRS_DS PROPERTY CXX_STANDARD_REQUIRED ON)