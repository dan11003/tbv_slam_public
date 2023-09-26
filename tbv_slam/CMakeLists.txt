cmake_minimum_required(VERSION 2.8.3)
project(tbv_slam)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE release)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden")
set(PYBIND11_PYTHON_VERSION 3.8 CACHE STRING "")
find_package(pybind11 REQUIRED)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    #    cmake_modules
    laser_geometry
    eigen_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    genmsg
    tf_conversions
    cfear_radarodometry
    place_recognition_radar
    alignment_checker
    )

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
        cfg/OptimizationParams.cfg
)

#add_subdirectory(pybind11)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

#find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen3 #libpcl-all-dev
    CATKIN_DEPENDS  roscpp laser_geometry sensor_msgs tf_conversions tf geometry_msgs  eigen_conversions pcl_ros  cfear_radarodometry place_recognition_radar alignment_checker
    LIBRARIES ${PROJECT_NAME}
    )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pose_graph_3d_error_term.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



set (${PROJECT_NAME}_LIB_SRCS
    src/${PROJECT_NAME}/loopclosure.cpp
    src/${PROJECT_NAME}/ceresoptimizer.cpp
    src/${PROJECT_NAME}/pose_graph_optimization.cpp
    src/${PROJECT_NAME}/posegraph.cpp
    src/${PROJECT_NAME}/tbv_slam.cpp
    src/${PROJECT_NAME}/utils.cpp
    )

#link_directories(${OCTOMAP_LIBRARY_DIRS})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${CERES_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${CERES_LIBRARIES} ${Boost_LIBRARIES} boost_serialization  pybind11::embed)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

#add_executable(lidar_odom_node src/lidar_odom_node.cpp)
#target_link_libraries(lidar_odom_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})



#add_executable(graph_optimizer_node src/graph_optimizer_node.cpp)
#target_link_libraries(graph_optimizer_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES})

add_executable(tbv_slam_offline src/tbv_slam_offline.cpp)
target_link_libraries(tbv_slam_offline ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} )
add_dependencies(tbv_slam_offline ${PROJECT_NAME}_gencfg)

add_executable(tbv_slam_online src/tbv_slam_online.cpp)
target_link_libraries(tbv_slam_online ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES} )
add_dependencies(tbv_slam_online ${PROJECT_NAME}_gencfg)

add_executable(odometry_training_node src/odometry_training_node.cpp)
target_link_libraries(odometry_training_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CERES_LIBRARIES})
