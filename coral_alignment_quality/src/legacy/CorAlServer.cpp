
//#include "graph_map/graph_map_fuser.h"
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
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include <time.h>
#include <fstream>
#include <cstdio>

#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "robust_mapping_custom_msgs/registration.h"
#include "robust_mapping_custom_msgs/n_registration.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "alignment_checker/utils.h"
#include "alignment_checker/scan.h"
#include "alignment_checker/scancomparsion.h"
/** \brief A ROS node which implements scan matching based on ndt
 * \author Daniel adolfsson
 *
 */

using std::string;
using std::cout;
using std::cerr;
using std::endl;
namespace ac=alignment_checker;

class RegistrationService{
public:
  RegistrationService():nh_("~"){

    double swell, max_swell_dist;
    nh_.param<std::string>("alignment_quality_service", alignment_quality_service, "/alignment_quality_service");
    nh_.param<std::string>("world_frame", world_id_, "map");
    nh_.param<double>("resolution", resolution_, 0.3);
    nh_.param<double>("max_swell_distance", ac::ScanType::max_swell_dist ,50);
    nh_.param<double>("max_swell", ac::ScanType::max_swell_dist, 2.5);
    nh_.param<bool>("downsample", downsample_, false);
    nh_.param<double>("rejection", rejection_, 0.1);
    nh_.param<bool>("visualize", visualize, true);
    ac::ScanType::max_swell = swell;
    ac::ScanType::max_swell_dist = max_swell_dist;
    service = nh_.advertiseService(alignment_quality_service, &RegistrationService::CorAlService, this);


  }
  void Publish(const std::string& topic, pcl::PointCloud<pcl::PointXYZ> &cloud, std::string frame_id, const ros::Time &t){
    cloud.header.frame_id = frame_id;
    pcl_conversions::toPCL(t, cloud.header.stamp);
    alignment_checker::PublishCloud(topic, cloud);
  }


  bool CorAlService(robust_mapping_custom_msgs::registration::Request& request, robust_mapping_custom_msgs::registration::Response& response){

    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(request.cloudsrc, *src);
    pcl::fromROSMsg(request.cloudtarget, *target);
    cout<<"src: "<<src->points.size()<<endl;
    cout<<"target: "<<target->points.size()<<endl;
    Eigen::Affine3d Tsrc, Ttar;
    tf::poseMsgToEigen(request.Tsrc, Tsrc);
    tf::poseMsgToEigen(request.Ttarget, Ttar);
    src->sensor_origin_(0) = Tsrc.translation()(0);
    src->sensor_origin_(1) = Tsrc.translation()(1);
    src->sensor_origin_(2) = Tsrc.translation()(2);

    target->sensor_origin_(0) = Ttar.translation()(0);
    target->sensor_origin_(1) = Ttar.translation()(1);
    target->sensor_origin_(2) = Ttar.translation()(2);



    ros::Time t = ros::Time::now();
    Publish("coral_target", *target, world_id_, t);
    Publish("coral_src", *src, world_id_, t);
    response.quality=1;
    ac::ScanComparsion comp(src, target, radius_, downsample_, ac::entropy, rejection_);
    bool status = false;
    double aligned_merged = 0, aligned_sep = 0;
    //comp.GetAlignmentQualityExtended(aligned, aligned_merged, aligned_sep);
   // c


    return true;

  }


  ros::NodeHandle nh_;
  ros::ServiceServer service;
  bool visualize;
  double resolution_;
  std::string alignment_quality_service;
  std::string world_id_;
  bool disable_registration_;
  double radius_, rejection_;
  bool downsample_ = false;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "coral_node");
  RegistrationService srv;
  ros::spin();

  return 0;
}



