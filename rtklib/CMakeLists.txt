cmake_minimum_required(VERSION 2.6)

set(src ${CMAKE_CURRENT_SOURCE_DIR}/src/src)

find_package(LAPACK OPTIONAL)
find_package(BLAS OPTIONAL)

if(LAPACK_FOUND and BLAS_FOUND)
    add_definitions(-DLAPACK)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/rtklib)

add_library(rtk STATIC
  ${src}/rtksvr.c
  ${src}/rtkcmn.c
  ${src}/rtkpos.c
  ${src}/geoid.c
  ${src}/solution.c
  ${src}/lambda.c
  ${src}/sbas.c
  ${src}/stream.c
  ${src}/rcvraw.c
  ${src}/rtcm.c
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
  ${src}/ephemeris.c
  ${src}/rinex.c
  ${src}/ionex.c
)
target_link_libraries(rtk ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
