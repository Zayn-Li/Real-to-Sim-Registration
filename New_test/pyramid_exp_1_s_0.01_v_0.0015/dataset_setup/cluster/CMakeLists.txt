cmake_minimum_required(VERSION 3.1...3.15)
#project( mytest )
#project( volume )
project( cluster)

#find_package(Python COMPONENTS Interpreter Development)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )

if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()

endif()

# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()

endif()

find_package(PCL 1.2 REQUIRED)
#include_directories("./include")
find_package(Eigen3 3.2.0) #(requires 3.2.0 or greater)
include(${CGAL_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(src)

#add_executable (mytest test.cpp)

#target_link_libraries (mytest PRIVATE ${PCL_LIBRARIES})
#target_link_libraries (mytest PRIVATE -L./ -ltet)
#target_link_libraries (mytest PRIVATE LibsModule)
#target_link_libraries (mytest PRIVATE -lboost_system)
#target_link_libraries(mytest PRIVATE ${CGAL_LIBS})

#add_executable (volume vol_mesh.cpp)

#target_link_libraries (volume PRIVATE ${PCL_LIBRARIES})
#target_link_libraries (volume PRIVATE -L./ -ltet)
#target_link_libraries (volume PRIVATE LibsModule)
#target_link_libraries (volume PRIVATE -lboost_system)
#target_link_libraries(volume PRIVATE ${CGAL_LIBS})

add_executable (cluster clustering.cpp)

target_link_libraries (cluster PRIVATE ${PCL_LIBRARIES})
target_link_libraries (cluster PRIVATE -L./ -ltet)
target_link_libraries (cluster PRIVATE LibsModule)
target_link_libraries (cluster PRIVATE -lboost_system)
target_link_libraries(cluster PRIVATE ${CGAL_LIBS})

