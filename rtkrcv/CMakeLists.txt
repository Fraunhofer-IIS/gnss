cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

################################################################################
# RTKLIB
################################################################################

#INCLUDE($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

rosbuild_find_ros_package(rtklib)
include_directories(${rtklib_PACKAGE_PATH}/include)
find_library(LIBRTK NAMES rtk PATHS ${rtklib_PACKAGE_PATH}/lib)
message(${LIBRTK})

rosbuild_add_executable(${PROJECT_NAME}
                       src/ros_options.cpp
                       src/rtkrcv.cpp
                       src/main_aisehe.cpp
                      )
target_link_libraries(${PROJECT_NAME} ${LIBRTK})
