cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(visualization)
#project(registration)

find_package(PCL 1.2 REQUIRED)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-g)

include_directories(${PCL_INCLUDE_DIRS} /usr/include/pcl-1.8/)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (registration registration.cpp)
#target_link_libraries (registration ${PCL_LIBRARIES})
add_executable (visualization visualize.cpp)
target_link_libraries (visualization ${PCL_LIBRARIES})
