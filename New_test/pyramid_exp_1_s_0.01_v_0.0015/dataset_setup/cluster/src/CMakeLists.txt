set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_LIBRARY(LibsModule 
    core.cpp
    platform.cpp
    maths.cpp
    mesh.cpp
    DeformativeObject3D.cpp
)
target_include_directories(LibsModule PUBLIC ../include)
