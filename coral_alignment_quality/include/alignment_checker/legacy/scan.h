#ifndef POINTSCORE_H
#define POINTSCORE_H

#include "stdio.h"
#include "iostream"
#include <vector>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <Eigen/Eigenvalues>
#include "Eigen/Dense"

#include "ros/ros.h"

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>
#include "pcl/common/centroid.h"
#include "math.h"
#include "pcl/filters/voxel_grid.h"
#include "cmath"
namespace alignment_checker{
using std::cout;
using std::cerr;
using std::endl;
typedef enum measurement_type{entropy,entropy_median,determinant, ndtp2d,rel_ndtp2d, mme}m_type;

  std::string mtype2string(m_type &type);

  m_type string2mtype(std::string &type);

class ScanType
{
public:



  ScanType(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, m_type type);

  virtual void SetInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);

  virtual void ComputeInformation(const double r=0.5, bool det_only = false, double d_factor = 0);

  double GetInformation(){return tot_entropy_;}

  virtual size_t GetInformationPointsetSize(){return entropy_pointset_size_;}

  void GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double r, std::vector<int> &idx){}

  void GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double r, std::vector<int> &idx_input, std::vector<int> &idx_this );

  //!
  //! \brief GetOverlap
  //! \return points in the scan which overlap with the input
  //!
  virtual pcl::PointCloud<pcl::PointXYZ>::Ptr GetOverlap(const ScanType &scan){return NULL;}

  void Update();

  void GetOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_src, double r=0.5);

  void ExtractIndecies(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, std::vector<int> &indecies, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered);

  void FilterMinEntropyLimit(const double min_entropy_th);

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetScanWithInformation();

  std::vector<double> GetEntropy(bool valid_only = false);

  std::vector<bool>& GetValidIndices(){return valid_pnt_;}

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetScan(){return cloud_;}

  double FilterEntropyByIndex(std::vector<bool>& indicies);

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetLocalCloud(pcl::PointXYZ p, double radius);


 static double median(std::vector<double> &scores);

 static double mean(std::vector<double> &scores, size_t &count);

 static double max_swell, max_swell_dist;

protected:

  ScanType();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;


  std::vector<double> entropy_; //range 0 to 1

  std::vector<bool> valid_pnt_; // if entropy is calculated for the point

  const double default_entropy_ = -100;

  double tot_entropy_ = 0;

  size_t entropy_pointset_size_ = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_with_information_;

  m_type type_;







};



}
#endif // POINTSCORE_H
