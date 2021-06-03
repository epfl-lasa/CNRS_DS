cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(CNRS_DS LANGUAGES CXX VERSION 1.0.0)

find_package(Eigen3 QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(Eigen3_FOUND)
  set(FIND_EIGEN "find_package(Eigen3 REQUIRED NO_CMAKE_PACKAGE_REGISTRY)")
else()
  find_package(Eigen3 MODULE REQUIRED)
  set(FIND_EIGEN "find_package(Eigen3 MODULE REQUIRED)")
endif()

set(CNRS_DS_HDR
  include/cnrs_ds/calculate_alpha.h
  include/cnrs_ds/modulated_DS.h
  include/cnrs_ds/nominal_DS.h
)

set(CNRS_DS_SRC
  src/calculate_alpha.cpp
  src/modulated_DS.cpp
  src/nominal_DS.cpp
)

add_library(CNRS_DS SHARED ${CNRS_DS_SRC} ${CNRS_DS_HDR})
target_include_directories(CNRS_DS PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
if(TARGET Eigen3::Eigen)
  target_link_libraries(CNRS_DS PUBLIC Eigen3::Eigen)
else()
  target_include_directories(CNRS_DS SYSTEM INTERFACE "${EIGEN3_INCLUDE_DIR}")
endif()

install(FILES ${CNRS_DS_HDR} DESTINATION include/cnrs_ds)
install(TARGETS CNRS_DS
  EXPORT CNRS_DS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(CNRS_DS_sample src/main.cpp)
target_link_libraries(CNRS_DS_sample PUBLIC CNRS_DS)

add_subdirectory(cmake)
