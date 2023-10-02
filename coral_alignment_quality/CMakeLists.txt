cmake_minimum_required(VERSION 2.8.3)
project(alignment_checker)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)

IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(LIB_SUFFIX "")
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 4)
    SET(LIB_SUFFIX 64)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 4)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")
set(PYBIND11_PYTHON_VERSION 3.8 CACHE STRING "")
find_package(pybind11 REQUIRED) 


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED
    geometry_msgs
    cmake_modules
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf_conversions
    tf
    genmsg
    cv_bridge
    tf_conversions
    cfear_radarodometry
    #ndt_map
    #robust_mapping_custom_msgs
    message_generation
    )

add_service_files(
  FILES
  AlignmentData.srv
)

generate_messages(
  DEPENDENCIES
)


find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs eigen_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf_conversions tf genmsg tf_conversions cfear_radarodometry message_generation #cmake_modules #ndt_map
)

INCLUDE_DIRECTORIES ( $ENV{EIGEN3_INCLUDE_DIR}  ${OpenCV_INCLUDE_DIRS})

include_directories( include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set (${PROJECT_NAME}_LIB_SRCS
    src/alignment_checker/AlignmentQuality.cpp
    src/alignment_checker/ScanEvaluator.cpp
    src/alignment_checker/DataHandler.cpp
    src/alignment_checker/Utils.cpp
    src/alignment_checker/ScanType.cpp
    src/alignment_checker/alignmentinterface.cpp
    #src/alignment_checker/scan.cpp
    #src/alignment_checker/scancomparsion.cpp
    #src/alignment_checker/alignmenttester.cpp
    #src/alignment_checker/viewer.cpp
    #src/alignment_checker/ndtScanComparsion.cpp ##uncomment if ndt libraries are not being used, this is only used for evaluation
    )
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR}  pcl_common ${OpenCV_LIBS}  pybind11::embed)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
#add_definitions(${PCL_DEFINITIONS})
#link_directories(${PCL_LIBRARY_DIRS})
add_executable(evaluate_scans src/evaluate_scans.cpp )
target_link_libraries(evaluate_scans ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS python/alignment_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_gtest(${PROJECT_NAME}_python_classifier_test test/python_classifier_interface_tests.cpp)
catkin_add_gtest(${PROJECT_NAME}_scan_learning_test test/scan_learning_interface_tests.cpp)

#if(TARGET ${PROJECT_NAME}_python_classifier_test)
  target_link_libraries(${PROJECT_NAME}_python_classifier_test ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})
#endif()

#if(TARGET ${PROJECT_NAME}_scan_learning_test)
  target_link_libraries(${PROJECT_NAME}_scan_learning_test ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})
#endif()


#add_executable(test_align src/test_alignment.cpp )
#target_link_libraries(test_align ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_executable(score_viewer src/score_viewer.cpp )
#target_link_libraries(score_viewer ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_executable(function_surface src/function_surface.cpp )
#target_link_libraries(function_surface ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_executable(CorAlServer src/CorAlServer.cpp )
#target_link_libraries(CorAlServer ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  $ENV{EIGEN3_INCLUDE_DIR} ${PROJECT_NAME} ${PCL_LIBRARIES})

#add_definitions(${PCL_DEFINITIONS})
