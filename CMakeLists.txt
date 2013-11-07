cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

PROJECT ( sure CXX C )

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE true)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#include(FindPCL.cmake)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(Boost_DEBUG 1)
FIND_PACKAGE ( Boost REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include(pcl_find_sse.cmake)
PCL_CHECK_FOR_SSE()

include_directories(include src)
add_definitions(-fpermissive)

set(sure_sources

    include/sure/data/typedef.h
    src/sure/data/typedef.cpp

    include/sure/access/point.h
    src/sure/access/point.cpp
    include/sure/access/region.h
    src/sure/access/region.cpp
    
    include/sure/data/map2d.h
    src/sure/data/map2d.cpp
    include/sure/data/configuration.h
    src/sure/data/configuration.cpp
    
    include/sure/memory/fixed_size_allocator.h
    src/sure/memory/fixed_size_allocator.cpp
    include/sure/memory/fixed_size_allocator_direct_access.h
    src/sure/memory/fixed_size_allocator_direct_access.cpp

    include/sure/normal/normal.h
    src/sure/normal/normal.cpp
    include/sure/normal/base_histogram.h
    src/sure/normal/base_histogram.cpp
    include/sure/normal/normal_histogram.h
    src/sure/normal/normal_histogram.cpp
    include/sure/normal/cross_product_histogram.h
    src/sure/normal/cross_product_histogram.cpp
    include/sure/normal/normal_estimation.h
    src/sure/normal/normal_estimation.cpp

    include/sure/payload/payload.h
    src/sure/payload/payload.cpp
    include/sure/payload/payload_xyzrgb.h
    src/sure/payload/payload_xyzrgb.cpp
    include/sure/payload/payload_normal.h
    src/sure/payload/payload_normal.cpp
    include/sure/payload/payload_cross_product.h
    src/sure/payload/payload_cross_product.cpp
    include/sure/payload/payload_entropy.h
    src/sure/payload/payload_entropy.cpp
    
    include/sure/data/range_image.h
    src/sure/data/range_image.cpp

    include/sure/octree/octree_node.h
    src/sure/octree/octree_node.cpp
    include/sure/octree/octree_node_list.h
    src/sure/octree/octree_node_list.cpp
    include/sure/octree/octree_level_map.h
    src/sure/octree/octree_level_map.cpp
    include/sure/octree/octree.h
    src/sure/octree/octree.cpp
    
    include/sure/keypoints/keypoint_calculation.h
    src/sure/keypoints/keypoint_calculation.cpp    
    
    include/sure/descriptor/histogram.h
    src/sure/descriptor/histogram.cpp
    include/sure/descriptor/color_descriptor.h
    src/sure/descriptor/color_descriptor.cpp
    include/sure/descriptor/lightness_descriptor.h
    src/sure/descriptor/lightness_descriptor.cpp
    include/sure/descriptor/shape_descriptor.h
    src/sure/descriptor/shape_descriptor.cpp    
    include/sure/descriptor/descriptor.h
    src/sure/descriptor/descriptor.cpp    
     
    include/sure/feature/feature.h
    src/sure/feature/feature.cpp
    include/sure/feature/feature_extraction.h
    src/sure/feature/feature_extraction.cpp
    
    include/sure/sure.h
    src/sure/sure.cpp    
    )

ADD_LIBRARY( ${PROJECT_NAME} SHARED ${sure_sources} )
target_link_libraries( ${PROJECT_NAME} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sure_example src/example.cpp)
target_link_libraries(sure_example ${PROJECT_NAME} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
