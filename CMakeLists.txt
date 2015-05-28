cmake_minimum_required(VERSION 2.8.3)
project(frame_registration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  ekz_public_lib
)


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
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${freenect2_INCLUDE_DIRS}
  ${ekz_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
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
    #/usr/include/vtk-5.8
    /usr/include/opencv2
    /usr/include/opencv
)

## Declare a cpp library
# add_library(cloud_preparation
#   src/${PROJECT_NAME}/cloud_preparation.cpp
# )

#add_library(frame_registration
#src/frame_registration.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/Map/Map3D.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/Map/Map3Dbow.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/FeatureDescriptor/FeatureDescriptor.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/FeatureDescriptor/OrbFeatureDescriptor.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/FeatureDescriptor/SurfFeatureDescriptor64.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/FeatureDescriptor/SurfFeatureDescriptor128.cpp
#/home/tmrcv1/catkin_ws/src/strands_3d_mapping/ekz-public-lib/src/FeatureDescriptor/FloatHistogramFeatureDescriptor.cpp
#)

## Declare a cpp executable
add_executable(frame_registration_node src/frame_registration.cpp)
add_executable(frame_registration_node2 src/frame_registration2.cpp)
add_executable(frame_registration_node3 src/frame_registration3.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cloud_preparation_node cloud_preparation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(frame_registration_node
  ${catkin_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${PCL_LIBRARIES}
  ${ekz_LIBRARY}
)

target_link_libraries(frame_registration_node2
  ${catkin_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${PCL_LIBRARIES}
  ${ekz_LIBRARY}
)

target_link_libraries(frame_registration_node3
  ${catkin_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${freenect2_LIBRARY}
  ${PCL_LIBRARIES}
  ${ekz_LIBRARY}
)






