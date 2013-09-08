cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

find_package(LAPACK)
find_package(BLAS)

if(${LAPACK_FOUND} AND ${BLAS_FOUND})
    add_definitions(-DLAPACK)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/rtklib)
add_definitions(-DDLL)

set(src ${CMAKE_CURRENT_SOURCE_DIR}/src/src)
file(GLOB RTK_SOURCES "${src}/*.c" "${src}/rcv/*.c")
rosbuild_add_library(rtk ${RTK_SOURCES})
target_link_libraries(rtk ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
