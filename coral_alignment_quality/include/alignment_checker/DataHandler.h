#pragma once

#include "alignment_checker/ScanType.h"
#include "boost/shared_ptr.hpp"

//Added by manuel
//From: https://github.com/ethz-asl/lidar_align/issues/16

/**ONLY NEEDED IN MY PC
#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
#include "rosbag/bag.h"
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue
**/


#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
#include <cv_bridge/cv_bridge.h>
#include "cfear_radarodometry/statistics.h"
//Coral includes
#include "alignment_checker/Utils.h"
#include "alignment_checker/ScanType.h"
#include "alignment_checker/AlignmentQuality.h"

namespace CorAlignment {

typedef std::pair<Eigen::Affine3d,ros::Time> poseStamped;
using namespace CFEAR_Radarodometry;


class dataHandler
{
public:
  dataHandler():nh_("~") {}

  virtual std::shared_ptr<PoseScan> Next() = 0;
protected:
  ros::NodeHandle nh_;
  Eigen::Affine3d TposePrev;

};

class FiledataHandler: public dataHandler
{
public:
  FiledataHandler(const std::string& directory, const std::string& poses, const std::string& data_prefix="", int index_first_scan = 0);

  std::shared_ptr<PoseScan> Next();
};

class RadarRosbagHandler: public dataHandler
{
public:
    RadarRosbagHandler(const std::string& rosbag_path, const PoseScan::Parameters& scanPars, const int rosbag_offset = 0, const double min_distance = -1.0,  const std::string& gt_topic = "/gt", const std::string& radar_topic = "/Navtech/Polar");

public:

  std::shared_ptr<PoseScan> Next();

protected:

  void UnpackImage(sensor_msgs::ImageConstPtr& image_msg);

  void UnpackPose(nav_msgs::Odometry::ConstPtr& odom_msg);

  const PoseScan::Parameters scanPars_;

  rosbag::Bag bag_;
  std::unique_ptr<rosbag::View> view_image_, view_pose_;
  rosbag::View::iterator m_image_, m_pose_;
  double min_distance_;
  bool synced_ = false;


  std::vector<cv_bridge::CvImagePtr> radar_stream_;
  std::vector<poseStamped> pose_stream_;
  ros::Publisher pub_odom, pub_image;
};


class MockupHandler: public dataHandler
{
public:
  MockupHandler();

  std::shared_ptr<PoseScan> Next();
private:
  int step = 1;
  const int step_resolution = 2;
  pcl::PointCloud<pcl::PointXYZI> cloud;
};


typedef std::unique_ptr<dataHandler> dataHandler_U;



}
