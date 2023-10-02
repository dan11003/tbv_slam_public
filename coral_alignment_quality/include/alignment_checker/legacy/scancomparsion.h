#ifndef SCANCOMPARISON_H
#define SCANCOMPARISON_H
#include "alignment_checker/scan.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "iostream"
#include "tf/transform_broadcaster.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
namespace alignment_checker {
using std::endl;
using std::cout;
using std::cerr;

class ScanComparsion
{

public:
  ScanComparsion():target_(new pcl::PointCloud<pcl::PointXYZ>),src_(new pcl::PointCloud<pcl::PointXYZ>){}

  ScanComparsion(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, double radius =0.5, bool downsample = true, m_type type = entropy, double reject_ratio = 0.1);

  //void SetNext(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetSrc(){return src_;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetTar(){return target_;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetSrcOverlap(){if(st_src_overlap!=NULL) return st_src_overlap->GetScanWithInformation(); else return NULL;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetTarOverlap(){if(st_tar_overlap!=NULL) return st_tar_overlap->GetScanWithInformation(); else return NULL;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetMergedOverlap(){if(st_merged!=NULL) return st_merged->GetScanWithInformation(); else return NULL;}

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetMergedDifferential();



  void StoreComparsionData(const std::string &dir, const std::string& suffix);

  virtual double GetAlignmentQuality(bool &aligned);

  virtual double GetAlignmentQualityExtended(bool &aligned, double &merged, double &separate);

  //double GetScore(){return merged_ - separate_;}

  //double GetScoreExt(double &separate, double &merged){ separate = separate_; merged_ = merged; return GetScore();}

  boost::shared_ptr<ScanType> GetMergedScan(){return st_merged;}

  static bool verbose;

protected:

  void CalculateOverlap();

  virtual bool CheckAlignment();

  virtual void SetInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target);


  pcl::PointCloud<pcl::PointXYZ>::Ptr src_, target_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_overlap_, src_overlap_;
  boost::shared_ptr<ScanType> ScanSrc_, ScanTarget_;

  boost::shared_ptr<ScanType> st_src_overlap = NULL;
  boost::shared_ptr<ScanType> st_tar_overlap = NULL;
  boost::shared_ptr<ScanType> st_merged = NULL;
  Eigen::Vector4f center_;
  double radius_;
  bool aligned_;
  double merged_;
  double separate_;
  double reject_ratio_;
  measurement_type type_;
  bool det_only_ = false;
  bool downsample_;
  double filter_distance = 0.08;
  double min_overlap_ = 0.1;

  static double tolerance;


};

typedef boost::shared_ptr<ScanComparsion> bstScanComp;

class VisComparsion
{
public:

  VisComparsion(const std::string frameid="/world");

  void PlotClouds(boost::shared_ptr<ScanComparsion> &comp);

  void PlotPoses(Eigen::Affine3d &src, Eigen::Affine3d &target);


private:


  pcl::PointCloud<pcl::PointXYZ>::Ptr Stamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time &tcloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Stamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const ros::Time &tcloud);

  ros::NodeHandle nh_;
  ros::Publisher pub_src, pub_tar, pub_overlap_src, pub_overlap_tar,pub_overlap_merged, pub_diff_merged;
  tf::TransformBroadcaster tf_br;
  std::string frameid_;
  bool downsample_;

};

}
#endif // SCANCOMPARISON_H
