

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

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <time.h>
#include <fstream>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/program_options.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/filter.h"
//#include "pcl/visualization/cloud_viewer.h"
#include "pcl/point_cloud.h"
#include "alignment_checker/scancomparsion.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/alignmenttester.h"
#include "alignment_checker/utils.h"
#include "alignment_checker/viewer.h"
#include "iostream"
#define EPS 0.00001
namespace po = boost::program_options;
namespace ac = alignment_checker;
using std::cout;
using std::endl;
using std::string;

using std::vector;

class ScoreSurface
{
public:
  ScoreSurface(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds,std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> &poses, double radius, double rejection){
    radius_ = radius;
    offset_<<0, 0, 0, 0, 0, 0;
    downsample_ = true;
    rejection_ = rejection;
    poses_ = poses;
    clouds_ = clouds;
  }
  void ComputeSurface(std::vector<double> &min, std::vector<double> &max, std::vector<double> &inc, std::vector<std::vector<double>> &surface){
    for(int i=0;i<min.size();i++){
      if(inc[i]<EPS)
        inc[i]=100000;
    }
    cout<<"Compute surface with arguments: \nMin: "<<endl;
    for(auto& m : min) cout<<-m<<", ";
    cout<<endl<<"Max: ";
    for(auto& m : max) cout<<m<<", ";
    cout<<endl<<"Inc: ";
    for(auto& m : inc) cout<<m<<", ";

    for(double i = -min[0];i<max[0]+EPS;i+=inc[0]){
      for(double j = -min[1] ; j<max[1]+EPS ; j+=inc[1]){
        for(double k = -min[2] ; j<max[1]+EPS ; j+=inc[1]){
          for(double l = -min[3] ; l<max[3]+EPS ; l+=inc[3]){
            for(double m = -min[4] ; m<max[4]+EPS ; m+=inc[4]){
              for(double n = -min[5] ; n<max[5]+EPS ; n+=inc[5]){
                if(!ros::ok()){
                  cerr<<"exit"<<endl;
                  exit(0);
                }

                offset_<<i,j,k,l,m,n;
                SetScans();
                bool aligned;
                double Q = comp_->GetAlignmentQuality(aligned);
                std::vector<double> pnt{i,j,k,l,m,n,Q};
                surface.push_back(pnt);
              }
            }
          }
        }

      }
    }
  }
private:


  void SetScans(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);

    if(target_idx_+1>=clouds_.size()|| target_idx_ <0 ||clouds_[target_idx_]==NULL ||clouds_[target_idx_+1]==NULL ){
      cerr<<"index out of range"<<endl;
      return;
    }

    target = clouds_[target_idx_];
    Eigen::Affine3d Toffset = poses_[target_idx_+1]*ac::VectorToAffine3d(offset_)*poses_[target_idx_+1].inverse();
    pcl::transformPointCloud(*clouds_[target_idx_+1], *src, Toffset);

    comp_ = ac::bstScanComp(new ac::ScanComparsion(src, target, radius_, downsample_, ac::entropy, rejection_));
    vis_.PlotClouds(comp_);
  }

  bool new_input_ = false;
  double step_ = 0.05;
  bool downsample_=true;
  int target_idx_ = 0;
  double rejection_ = 0;
  Eigen::Matrix<double,6,1> offset_;
  double radius_ = 0.5;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> poses_;
  ac::bstScanComp comp_;
  ac::VisComparsion vis_;

  std::thread *input_th_;

};

int main(int argc, char **argv)
{


  string filepath, directory_clouds, output_filename;
  double ground_height, radius;
  bool segment_ground, visualize, identity;
  int index_first_scan;
  double min_distance;
  ros::init(argc, argv, "score_viewer");
  ros::NodeHandle param("~");
  double max_swell_distance, max_swell, rejection_ratio;
  ros::Publisher cloud_pub;
  cloud_pub =param.advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
  po::options_description desc("Allowed options");
  std::string cloud_prefix;
  desc.add_options()
      ("help", "produce help message")
      ("pose-file-path",po::value<std::string>(&filepath)->default_value(std::string("/home/daniel/.ros/maps/offarla-2012_gt=1_submap=0_sizexy=250_Z=15_intrchR=8_compR=0_res=0.4_maxSensd=130_keyF=1_d=0.3_deg=3_alpha=0_dl=0_xyzir=0_mpsu=0_mnpfg=6kmnp0_sensorpose_est.txt")),"file of path containing pose files")
      ("cloud-dir", po::value<std::string>(&directory_clouds)->default_value(std::string("/home/daniel/ros/mapping_ws/src/graph_map/graph_map/scripts")),"directory of clouds")
      ("cloud-prefix",po::value<std::string>(&cloud_prefix)->default_value(std::string("cloud_")),"prefix of cloud names")
      ("output-filename",po::value<std::string>(&output_filename)->default_value(std::string("eval.txt")),"output directory")
      ("index-first-scan",po::value<int>(&index_first_scan)->default_value(1),"index of first scan")
      ("radius", po::value<double>(&radius)->default_value(0.5),"radius of a voxel")
      ("Sinc",po::value<std::vector<double> >()->multitoken(), "increment")
      ("Smin",po::value<std::vector<double> >()->multitoken(), "min surf cpordinate")
      ("Smax",po::value<std::vector<double> >()->multitoken(), "max surface coordinate")
      ("min-distance", po::value<double>(&min_distance)->default_value(0.45),"min dinstance filtering")
      ("max-swell-distance", po::value<double>(&max_swell_distance)->default_value(50)," max_swell_distance")
      ("ent-reject-ratio", po::value<double>(&rejection_ratio)->default_value(0.0)," rejection ratio")
      ("max-swell", po::value<double>(&max_swell)->default_value(0)," max_swell")
      ("visualize","visualize pointclouds");



  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return false;
  }
  if(filepath.size()==0){
    std::cerr<<"File path empty"<<std::endl;
    exit(0);
  }
  std::vector<double> Sinc;
  if (!vm["Sinc"].empty() && (Sinc = vm["Sinc"].as<vector<double> >()).size() == 6) {
    // good to go
  }
  std::vector<double> Smin;
  if (!vm["Smin"].empty() && (Smin = vm["Smin"].as<vector<double> >()).size() == 6) {
    // good to go
  }
  std::vector<double> Smax;
  if (!vm["Smax"].empty() && (Smax = vm["Smax"].as<vector<double> >()).size() == 6) {
    // good to go
  }

  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses;
  ac::ReadPosesFromFile(filepath, poses);

  if(poses.size()==0){
    std::cerr<<"pose vector empty"<<std::endl;
    exit(0);
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds, filtered_clouds;
  cout<<"start index"<<index_first_scan<<endl;
  ac::ReadCloudsFromFile(directory_clouds, cloud_prefix, clouds, index_first_scan);
  if(poses.size()!=clouds.size()){
    std::cerr<<"Input data is of wrong size: clouds="<<clouds.size()<<", poses="<<poses.size()<<endl;
    exit(0);
  }
  else
    cout<<"Found "<<poses.size()<<" poses and point clouds"<<endl;
  ac::ScanComparsion::verbose = true;

  ac::FilterCloudsByDistance(clouds, poses, min_distance);
  ac::SetScanLocations(clouds,poses);
  ac::ScanType::max_swell = max_swell;
  ac::ScanType::max_swell_dist = max_swell_distance;
  std::vector<std::vector<double> > surface;
  ScoreSurface surf(clouds, poses, radius, rejection_ratio);
  surf.ComputeSurface(Smin, Smax, Sinc, surface);

  std::string filename = directory_clouds+"/"+output_filename;
  std::ofstream ofs (filename, std::ofstream::out);
  for (const auto& pnts : surface) {
    for(int i = 0 ; i < pnts.size()-1 ; i++){
      ofs<<pnts[i]<<", ";
    }
    ofs<<pnts[pnts.size()-1]<<endl;
  }
  cout<<"stored surface in file: "<<filename<<endl;





  return 0;
}



