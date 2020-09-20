cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#find_package(PythonInterp 3 REQUIRED)
#find_package(PythonLibs 3 REQUIRED)

#project(visualization)
project(registration)

#find_package(PCL 1.2 REQUIRED)
add_subdirectory(pybind11)
#find_package(pybind11 REQUIRED)
SET(CMAKE_BUILD_TYPE "Release")

#SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-g)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


pybind11_add_module(registration registration.cpp src/core.cpp src/maths.cpp src/mesh.cpp src/platform.cpp)
target_include_directories(registration PRIVATE "./include")
#target_link_libraries (registration PRIVATE pybind11::module ${PCL_LIBRARIES})
