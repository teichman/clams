cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(LIB_COMPILE_FLAGS -fopenmp -w)
#set(LIB_DEPENDENCIES "")
#set(LIB_BOOST_DEPENDENCIES "")

#set(BIN_COMPILE_FLAGS -fopenmp -w)
#set(BIN_DEPENDENCIES "")
#set(BIN_BOOST_DEPENDENCIES "")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# -- Everything in src/*cpp is a library file
FILE(GLOB LIB_CPP "${PROJECT_SOURCE_DIR}/src/lib/*.cpp")
set(HAS_LIBRARIES ${LIB_CPP})
IF(HAS_LIBRARIES)
  rosbuild_add_library(${PROJECT_NAME} ${LIB_CPP})
  IF(LIB_COMPILE_FLAGS)
    rosbuild_add_compile_flags(${PROJECT_NAME} ${LIB_COMPILE_FLAGS})
  ENDIF()
  IF(LIB_DEPENDENCIES)
    target_link_libraries(${PROJECT_NAME} ${LIB_DEPENDENCIES})
  ENDIF()
  IF(LIB_BOOST_DEPENDENCIES)
    rosbuild_link_boost(${PROJECT_NAME} ${LIB_BOOST_DEPENDENCIES})
  ENDIF()
ENDIF()

# -- Everything in src/prog/*cpp is a binary file
FILE(GLOB EXE_CPP "${PROJECT_SOURCE_DIR}/src/prog/*.cpp")
foreach(exec ${EXE_CPP})
  STRING(REGEX REPLACE "(.*)/src/prog/(.*).cpp" "\\2" basename "${exec}")
  rosbuild_add_executable(${basename} ${exec})
  IF(BIN_COMPILE_FLAGS)
    rosbuild_add_compile_flags(${basename} ${BIN_COMPILE_FLAGS})
  ENDIF()
  IF(HAS_LIBRARIES)
    target_link_libraries(${basename} ${PROJECT_NAME} ${BIN_DEPENDENCIES})
  ELSE()
    target_link_libraries(${basename} ${BIN_DEPENDENCIES})
  ENDIF()
  IF(BIN_BOOST_DEPENDENCIES)
    rosbuild_link_boost(${basename} ${BIN_BOOST_DEPENDENCIES})
  ENDIF()
endforeach(exec)

# -- Everything in src/test/*cpp is a gtest file
FILE(GLOB TEST_CPP "${PROJECT_SOURCE_DIR}/src/test/*.cpp")
foreach(tst ${TEST_CPP})
  STRING(REGEX REPLACE "(.*)/src/test/(.*).cpp" "\\2" basename "${tst}")
  rosbuild_add_gtest(${basename} ${tst})
  target_link_libraries(${basename} ${PROJECT_NAME})
endforeach(tst)
