cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

find_package(LAPACK)
find_package(BLAS)

#if(${LAPACK_FOUND} and ${BLAS_FOUND})
    add_definitions(-DLAPACK)
#endif()

rosbuild_genmsg()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/rtklib)
rosbuild_find_ros_package(rtklib)

set(src ${CMAKE_CURRENT_SOURCE_DIR}/src/src)
rosbuild_add_library(rtk
  ${src}/rtksvr.c
  ${src}/convkml.c
  ${src}/datum.c
  ${src}/download.c
  ${src}/postpos.c
  ${src}/ppp_ar.c
  ${src}/qzslex.c
  ${src}/rtcm3e.c
  ${src}/streamsvr.c
  ${src}/tle.c
  ${src}/rtkcmn.c
  ${src}/rtkpos.c
  ${src}/geoid.c
  ${src}/solution.c
  ${src}/lambda.c
  ${src}/sbas.c
  ${src}/stream.c
  ${src}/rcvraw.c
  ${src}/rtcm.c
  ${src}/rtcm2.c
  ${src}/rtcm3.c
  ${src}/preceph.c
  ${src}/options.c
  ${src}/pntpos.c
  ${src}/ppp.c
  ${src}/rcv/novatel.c
  ${src}/rcv/ublox.c
  ${src}/rcv/ss2.c
  ${src}/rcv/crescent.c
  ${src}/rcv/skytraq.c
  ${src}/rcv/gw10.c
  ${src}/rcv/javad.c
  ${src}/rcv/binex.c
  ${src}/rcv/nvs.c
  ${src}/rcv/rcvlex.c
  ${src}/ephemeris.c
  ${src}/rinex.c
  ${src}/ionex.c
)
target_link_libraries(rtk ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})