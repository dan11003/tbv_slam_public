#pragma once
#include <Eigen/Eigen>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "iostream"
#include "ros/ros.h"
#include <fstream>
#include <pcl/io/pcd_io.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/registration/transforms.h"
#include "pcl/common/distances.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "cfear_radarodometry/registration.h"
#include<opencv2/opencv.hpp>



namespace CorAlignment {

using std::cout;
using std::cerr;
using std::endl;
using std::string;

/*!
   \brief Extract features from polar radar data using the method described in cen_icra18
   \param fft_data Polar radar power readings
   \param zq If y(i, j) > zq * sigma_q then it is considered a potential target point
   \param sigma_gauss std dev of the gaussian filter uesd to smooth the radar signal
   \param min_range We ignore the range bins less than this
   \param targets [out] Matrix of feature locations (azimuth_bin, range_bin, 1) x N
*/
double cen2018features(cv::Mat fft_data, Eigen::MatrixXd &targets, float zq =  3.0, int sigma_gauss = 17, int min_range = 2.5);

/*!
   \brief Extract features from polar radar data using the method described in cen_icra19
   \param fft_data Polar radar power readings
   \param max_points Maximum number of targets points to be extracted from the radar image
   \param min_range We ignore the range bins less than this
   \param targets [out] Matrix of feature locations (azimuth_bin, range_bin, 1) x N
*/
double cen2019features(cv::Mat fft_data, Eigen::MatrixXd &targets, int max_points = 1000, int min_range = 0);

/*double cen2019descriptors(std::vector<double> azimuths, cv::Size polar_dims, Eigen::MatrixXd polar_points,
    Eigen::MatrixXd cart_targets, float radar_resolution, float cart_resolution, int cart_pixel_width,
    cv::Mat &descriptors, int navtech_version = CTS350);*/


void SetScanLocations(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds,  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses);

void SetScanLocation(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud,  Eigen::Affine3d &pose);

void ReadCloudsFromFile(const std::string directory, const std::string prefix, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, int start_index=0);

void ReadPosesFromFile(const std::string &filepath, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses);

Eigen::Affine3d TransRotvectorToAffine3d(const std::vector<double> &v);

Eigen::Affine3d VectorToAffine3d(const Eigen::Matrix<double, 6,1> &v);

Eigen::Affine3d VectorToAffine3dxyez(double x, double y, double theta);

Eigen::Affine3d VectorToAffine3dxyez(const std::vector<double>& vek);

void SegmentGround(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &filtered, double height);

void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  &cloud, pcl::PointCloud<pcl::Normal>::Ptr  &normal,float voxelsize=0.1);

void FilterClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &normals);

void FilterCloudsByDistance(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses, double radius);

pcl::PointCloud<pcl::PointXY>::Ptr pcl3dto2d(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, std::vector<double>& intensity);

pcl::PointCloud<pcl::PointXYZI>::Ptr pclAddIntensity(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const std::vector<double>& intensity);

double get_azimuth_index(const std::vector<double> &azimuths, double azimuth);

void radar_polar_to_cartesian(const cv::Mat &polar_in, const std::vector<double> &azimuths_in, cv::Mat &cart_out,
    float radar_resolution = 0.04328, float cart_resolution = 0.2384, int cart_pixel_width = 300, bool fix_wobble = false);

void RotoTranslation(const cv::Mat& input, cv::Mat& output, const Eigen::Affine3d& T, const float image_res = 0.04328);

cv_bridge::CvImagePtr CreateImage(cv_bridge::CvImagePtr ref);

void NormalizeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double min = 60);


template <typename T>
std::ostream & operator << (std::ostream & os, const std::vector<T> & vec)
{
    for(auto elem : vec)
    {
        os<<elem<< " ";
    }
    return os;
}

const std::string Vec2String(const std::vector<std::string>& vec);


}
