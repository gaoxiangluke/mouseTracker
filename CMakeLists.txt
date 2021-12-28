cmake_minimum_required(VERSION 2.8)
project(mouseTracker)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
# External libray
find_package(OpenCV 4.5.5 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
# glog

include_directories("/usr/include/eigen3")
set(THIRD_PARTY_LIBS g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension ${OpenCV_LIBS}  ${GLOG_LIBRARIES})

#source 
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(app)
