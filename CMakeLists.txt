cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(sure3d)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE true)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if(OPENMP_FOUND)
add_definitions("${OpenMP_CXX_FLAGS}")    
endif(OPENMP_FOUND)

find_package(Boost REQUIRED)
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )

include(pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()

include_directories(include src)

set(srcs
	src/sure/normal_histogram.cpp
	src/sure/normal_histogram_allocator.cpp
	src/sure/octree_value.cpp
	src/sure/configuration.cpp
	src/sure/surflet.cpp
	src/sure/color_surflet.cpp
	src/sure/map2d.cpp
	src/sure/range_image.cpp
	src/sure/histogram_l2.cpp
	src/sure/histogram_emd.cpp
	src/sure/hue_descriptor.cpp
	src/sure/point_feature_descriptor.cpp
	src/sure/lightness_descriptor.cpp
	src/sure/feature.cpp
	src/sure/sure_estimator.cpp
	)

add_library(${PROJECT_NAME} SHARED ${srcs})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 0.1) 

add_executable(test_sure src/test_sure.cpp)
target_link_libraries(test_sure ${PROJECT_NAME} ${PCL_LIBRARIES})

