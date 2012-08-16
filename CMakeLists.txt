cmake_minimum_required(VERSION 2.8)

find_package(CGAL REQUIRED)
find_package(PCL REQUIRED)

if(NOT CGAL_FOUND)
    message("CGAL NOT FOUND")
endif(NOT CGAL_FOUND)

if(NOT PCL_FOUND)
    message("PCL NOT FOUND")
endif(NOT PCL_FOUND)


##### all required package found #####

message("Using CGAL_USE_FILE @CGAL_USE_FILE@")
include(${CGAL_USE_FILE})

message("Using PCL")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(convex_hull convex_hull.cxx)

target_link_libraries(convex_hull ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

## install testCGAL.exe to bin
#install(TARGETS testCGAL RUNTIME DESTINATION bin)
