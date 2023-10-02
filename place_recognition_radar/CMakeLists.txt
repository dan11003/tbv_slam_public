    cmake_minimum_required(VERSION 2.8.3)
project(place_recognition_radar)

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)

set(PYBIND11_PYTHON_VERSION 3.8 CACHE STRING "")
find_package(pybind11 REQUIRED) 

IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(LIB_SUFFIX "")
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(LIB_SUFFIX 64)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 4)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED 
  roscpp
  tf
  nav_msgs
  rospy
  cv_bridge
  image_transport
  cfear_radarodometry
  alignment_checker
  pcl_ros
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf nav_msgs rospy cv_bridge image_transport pcl_ros pcl_conversions cfear_radarodometry
#  DEPENDS system_lib
)


INCLUDE_DIRECTORIES ( $ENV{EIGEN3_INCLUDE_DIR}  ${OpenCV_INCLUDE_DIRS})

include_directories( include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 /usr/include/pcl/
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Scancontext.cpp
  src/${PROJECT_NAME}/RadarScancontext.cpp
  src/${PROJECT_NAME}/EvaluationManager.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${OpenCV_LIBS})
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


