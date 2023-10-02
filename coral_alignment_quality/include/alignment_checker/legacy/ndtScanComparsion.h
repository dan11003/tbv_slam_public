#ifndef NDTSCANCOMPARISON_H
#define NDTSCANCOMPARISON_H
#include "alignment_checker/scan.h"
#include "alignment_checker/scancomparsion.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "iostream"
#include "tf/transform_broadcaster.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
#include "ndt_map/ndt_map.h"
namespace alignment_checker {

using std::endl;
using std::cout;
using std::cerr;

class NdtScanComparsion : public ScanComparsion
{

public:

  NdtScanComparsion(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, double radius =0.5, bool downsample = true, m_type type = entropy);


protected:

  bool CheckAlignment(double &tot_distance, int &overlap, double &avg_logdet, pcl::PointCloud<pcl::PointXYZ>::Ptr &scan, perception_oru::NDTMap* reference);

  void SetInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target);

  perception_oru::NDTMap *ndttar_, *ndtsrc_;

};


}
#endif // NDTSCANCOMPARISON_H
