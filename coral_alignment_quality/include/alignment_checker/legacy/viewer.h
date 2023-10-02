#ifndef VIEWER_H
#define VIEWER_H
#include "stdio.h"
#include "iostream"
#include "Eigen/Eigen"
#include "thread"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "alignment_checker/scancomparsion.h"
#include "ndtScanComparsion.h"
#include "aliases.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/utils.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/subscriber.h"
#include "ros/node_handle.h"
#include "visualization_msgs/Marker.h"
#include "pcl/common/common.h"
#include "pcl/common/centroid.h"
#include "math.h"
namespace alignment_checker {


class viewer
{
public:

  viewer(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> &poses, double radius, std::string &directory, measurement_type type, double ent_reject_ratio);

  void PointSub(const geometry_msgs::PointStamped::ConstPtr &msg);

private:

  void KeyboardInputThread();

  void SetScans();

  bool new_input_ = false;
  double step_ = 0.05;
  bool downsample_=true;
  int target_idx_ = 0;
  Eigen::Matrix<double,6,1> offset_;
  double radius_;
  double ent_reject_ratio_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> poses_;
  bstScanComp comp_;
  VisComparsion vis_;
  std::string directory_;
  std::thread *input_th_;
  measurement_type type_;
  ros::Subscriber  sub_;
  ros::NodeHandle nh_;
  ros::Publisher cloud_local_pub, covar_pub;
  geometry_msgs::PointStamped last_point;

};



}
#endif // VIEWER_H
