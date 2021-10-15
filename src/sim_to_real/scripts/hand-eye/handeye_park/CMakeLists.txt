cmake_minimum_required(VERSION 3.10)
project(handeye_park)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(thirdparty/Sophus)
include_directories(include)
 
set(SOURCES src/run_handeye_park.cpp)
 
add_executable(run_handeye_park ${SOURCES})