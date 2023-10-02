#ifndef ALIGNMENTTESTER_H
#define ALIGNMENTTESTER_H
#include "random"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/scancomparsion.h"
#include "alignment_checker/ndtScanComparsion.h"
#include "alignment_checker/utils.h"
namespace alignment_checker {


class AlignmentTester
{
public:

  AlignmentTester(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses, std::vector<int> &ignore, double radius=0.5, measurement_type type=entropy, bool downsample=true, double rejection_ratio = 0.1);

  //void PerformAndSaveTest(const std::string &dir);

  void PerformAndSaveTest(const std::string &dir, const std::string &filename, const std::string &dataset);

  void ReAlignScanFixedOffset(double d, double alpha);



protected:

  Eigen::Affine3d vectorToAffine3d(const Eigen::Matrix<double,6,1> &v);

  double GausianNoiseGen(double dev);

  void AllocateScans();

  void ReAlignScansNoOffset();


  double pos_offset, angle_offset;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses_;


  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_error_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > Terror_;
  std::vector<Eigen::Matrix<double,6,1>> offsets_;

  std::vector<int> ignore_idx_;
  ros::Publisher pub, pub2;
  alignment_checker::VisComparsion vis_;
  double radius_;
  measurement_type type_;
  std::string method;
  bool downsample_;
  double rejection_ratio_;

};

}
#endif // ALIGNMENTTESTER_H
