cmake_minimum_required(VERSION 2.8.3)
project(frame_registration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

find_package(freenect2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp rostime std_msgs sensor_msgs nodelet cv_bridge compressed_depth_image_transport depth_registration)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
 catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES frame_registration
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS system_lib
  DEPENDS
  PCL
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
#add_custom_target(additional_files_${PROJECT_NAME}
#  SOURCES
#  ${FILES_LIST}
#  ${PROJECT_SOURCE_DIR}/../README.md
#)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${freenect2_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if(DEPTH_REG_OPENCL)
  add_definitions(-DDEPTH_REG_OPENCL)
endif()
if(DEPTH_REG_CPU)
  add_definitions(-DDEPTH_REG_CPU)
endif()
add_definitions(-DK2_CALIB_PATH="${PROJECT_SOURCE_DIR}/data/")

INCLUDE_DIRECTORIES(
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/core
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/FeatureDescriptor
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/FeatureExtractor
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/FrameMatcher
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/Map
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/mygeometry
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/RGBDSegmentation
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include/TransformationFilter
/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/include
/usr/include/pcl-1.7
/usr/include/eigen3
/usr/include/vtk-5.8
/usr/include/opencv2
)


link_directories(/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib)

add_library(frame_registration_nodelet SHARED src/frame_registration.cpp /home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/Map/Map3D.cpp /home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/Map/Map3Dbow.cpp)
target_link_libraries(frame_registration_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${PCL_LIBRARIES}

)

add_executable(frame_registration src/frame_registration.cpp /home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/Map/Map3D.cpp /home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/Map/Map3Dbow.cpp)
target_link_libraries(frame_registration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${PCL_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS frame_registration frame_registration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_frame_registration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#find_package(catkin REQUIRED)
# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/pcl-1.7)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

