#pragma once
#include <ros/ros.h>


#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>

#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"

#include <time.h>
#include <fstream>
#include <cstdio>
#include "algorithm"
#include "map"



#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "tbv_slam/posegraph.h"
#include "cfear_radarodometry/types.h"
#include "robust_mapping_custom_msgs/registration.h"
#include "robust_mapping_custom_msgs/DetectLoop.h"

#include "ros/service.h"
#include "robust_mapping_custom_msgs/n_registration.h"
#include "tbv_slam/loopclosure.h"
#include "boost/shared_ptr.hpp"
namespace tbv_slam {

using namespace CFEAR_Radarodometry;

class TBVSLAM
{
  public:
  struct Parameters{



    loopclosure::BaseParameters loop_pars;

    void GetParametersFromRos( ros::NodeHandle& param_nh);

    std::string ToString(){
      std::ostringstream stringStream;
      stringStream << loop_pars.ToString();
      return stringStream.str();
    }
  };


  TBVSLAM( PoseGraphPtr graph, const TBVSLAM::Parameters& par = TBVSLAM::Parameters() );

  bool ProcessFrame(const bool optimize = true, const bool loopclosure = true); // No Multithreading

protected:

  const TBVSLAM::Parameters& par_;
  PoseGraphPtr graph_;
  std::vector<boost::shared_ptr<loopclosure>> loop_closures_;

};

}
