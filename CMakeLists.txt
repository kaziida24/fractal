# CMake file for fractal analysis

# Set CMake requirement 
cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

# Name project 
project(fractal_analysis)

# Find packages and set compiler flags 
find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
find_package(PCL 1.2 REQUIRED)

# Includes 
include_directories(include 
					${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( PROJECT_LINK_LIBS libfractal_lib.so)

LIST(APPEND LINK_LIBS
	${PCL_LIBRARIES})

LIST(APPEND SOURCEFILES 
		src/fractalAnalysis.cpp
		src/ascii2pcd_help.cpp)

ADD_LIBRARY(fractal_lib SHARED ${SOURCEFILES})

add_executable(fractal src/fractal.cpp)
target_link_libraries(fractal -lpthread ${LINK_LIBS})
target_link_libraries(fractal_lib -lpthread ${LINK_LIBS})

add_executable(visualize src/visualize.cpp)
target_link_libraries(visualize -lpthread ${LINK_LIBS})

add_executable(practice src/practice.cpp)
target_link_libraries(practice -lpthread ${LINK_LIBS})

add_executable(ascii2pcd src/ascii2pcd.cpp)
target_link_libraries(ascii2pcd -lpthread ${LINK_LIBS})

add_executable( fractal_practice src/fractal_practice.cpp)
target_link_libraries(fractal_practice -lpthread ${LINK_LIBS})

target_link_libraries(fractal -lpthread fractal_lib)
target_link_libraries(visualize -lpthread fractal_lib)
target_link_libraries(practice -lpthread fractal_lib)
target_link_libraries(ascii2pcd -lpthread fractal_lib)

# Copy Python file for plotting
CONFIGURE_FILE( "src/python/plot_fractal.py" "plot_fractal.py" 
	[COPYONLY])

# Copy MATLAB files for plotting and visualization 
CONFIGURE_FILE( "src/MATLAB/plot_fractal.m" "plot_fractal.m" [COPYONLY])

CONFIGURE_FILE( "src/MATLAB/visualizeDataset.m" "visualizeDataset.m" [COPYONLY])