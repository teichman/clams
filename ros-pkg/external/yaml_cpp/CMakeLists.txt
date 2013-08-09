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

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/binary.cpp
  src/convert.cpp
  src/directives.cpp
  src/emit.cpp
  src/emitfromevents.cpp
  src/emitter.cpp
  src/emitterstate.cpp
  src/emitterutils.cpp
  src/exp.cpp
  src/memory.cpp
  src/nodebuilder.cpp
  src/node.cpp
  src/node_data.cpp
  src/nodeevents.cpp
  src/null.cpp
  src/ostream_wrapper.cpp
  src/parse.cpp
  src/parser.cpp
  src/regex.cpp
  src/scanner.cpp
  src/scanscalar.cpp
  src/scantag.cpp
  src/scantoken.cpp
  src/simplekey.cpp
  src/singledocparser.cpp
  src/stream.cpp
  src/tag.cpp
  )

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
