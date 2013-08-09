cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(VTK REQUIRED)
if(VTK_FOUND)
  include(${VTK_USE_FILE})
endif()

#common commands for building c++ executables and libraries
rosbuild_add_library(clams
  src/lib/discrete_depth_distortion_model.cpp
  src/lib/discrete_depth_distortion_model_helpers.cpp
  src/lib/slam_calibrator.cpp
  src/lib/slam_calibration_visualizer.cpp
  src/lib/trajectory.cpp
  )

rosbuild_add_compile_flags(clams -fopenmp)
rosbuild_add_link_flags(clams -fopenmp)
rosbuild_add_boost_directories()
rosbuild_link_boost(clams program_options)

rosbuild_add_executable(inspect src/prog/inspect.cpp)
target_link_libraries(inspect clams)

rosbuild_add_executable(calibrate src/prog/calibrate.cpp)
target_link_libraries(calibrate clams)

rosbuild_add_executable(visualize_model src/prog/visualize_model.cpp)
target_link_libraries(visualize_model clams)

rosbuild_add_executable(convert_trajectory src/prog/convert_trajectory.cpp)
target_link_libraries(convert_trajectory clams)

rosbuild_add_executable(visualize_trajectory src/prog/visualize_trajectory.cpp)
target_link_libraries(visualize_trajectory clams)
