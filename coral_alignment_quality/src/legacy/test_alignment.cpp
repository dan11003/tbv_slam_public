

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
#include "tf_conversions/tf_eigen.h"
#include "alignment_checker/utils.h"
namespace po = boost::program_options;
namespace ac=alignment_checker;
using std::cout;
using std::endl;
using std::string;


//!
//! \brief vectorToAffine3d
//! \param v x y z ex yz ez
//! \return
//!





int main(int argc, char **argv)
{

  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > poses;
  string filepath, directory_clouds, output_dir;
  double ground_height, radius;
  bool  visualize, identity;
  double min_distance;
  int index_first_scan;
  std::string output_file_name, dataset;
  std::string method;
  double rejection_ratio;
  alignment_checker::measurement_type type = alignment_checker::entropy;
  ros::init(argc, argv, "clustering_node");
  ros::NodeHandle param("~");
  ros::Publisher cloud_pub;

  double max_swell_distance, max_swell;
  cloud_pub =param.advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
  po::options_description desc("Allowed options");
  std::string cloud_prefix;
  desc.add_options()
      ("help", "produce help message")
      ("pose-file-path",po::value<std::string>(&filepath)->default_value(std::string("/home/daniel/.ros/maps/offarla-2012_gt=1_submap=0_sizexy=250_Z=15_intrchR=8_compR=0_res=0.4_maxSensd=130_keyF=1_d=0.3_deg=3_alpha=0_dl=0_xyzir=0_mpsu=0_mnpfg=6kmnp0_sensorpose_est.txt")),"file of path containing pose files")
      ("cloud-dir", po::value<std::string>(&directory_clouds)->default_value(std::string("/home/daniel/ros/mapping_ws/src/graph_map/graph_map/scripts")),"directory of clouds")
      ("cloud-prefix",po::value<std::string>(&cloud_prefix)->default_value(std::string("cloud_")),"prefix of cloud names")
      ("output-dir",po::value<std::string>(&output_dir)->default_value(std::string("")),"output directory")
      ("output-file-name",po::value<std::string>(&output_file_name)->default_value(std::string("entropy")),"filename")
      ("method",po::value<std::string>(&method)->default_value(std::string("entropy")),"evaluation method")
      ("data-set",po::value<std::string>(&dataset)->default_value(std::string("dataset")),"filename")
      ("index-first-scan",po::value<int>(&index_first_scan)->default_value(1),"index of first scan")
      ("radius", po::value<double>(&radius)->default_value(0.5),"radius of a voxel")
      ("min-distance", po::value<double>(&min_distance)->default_value(0.45),"radius of a voxel")
      ("rejection-ratio", po::value<double>(&rejection_ratio)->default_value(0.1)," rejection ratio")
      ("max-swell-distance", po::value<double>(&max_swell_distance)->default_value(50)," max_swell_distance")
      ("max-swell", po::value<double>(&max_swell)->default_value(0)," max_swell")
      ("ignore-targets", po::value<std::vector<int> >()->multitoken(), "leave these out")
      ("visualize","visualize pointclouds")
      ("downsample","downsample")
      ("small-error","small error")
      ("medium-error","small error")
      ("determinant","use determinant")
      ("large-error","small error")
      ("segment-ground","segment gorund at a certain hight");

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

  visualize = vm.count("visualize");
  identity = vm.count("identity");
  bool small_error = vm.count("small-error");
  bool medium_error = vm.count("medium-error");
  bool large_error = vm.count("large-error");
  bool downsample = vm.count("downsample");
  std::vector<int> ignore_idx;
  if (vm.count("ignore-targets"))
  {
    ignore_idx = vm["ignore-targets"].as< std::vector<int> >() ;
    for(auto i:ignore_idx)
      cout<<", "<<i<<endl;

  }
  type = alignment_checker::string2mtype(method);

  if(!(large_error||medium_error||small_error)){
    cerr<<"need to specify error magniute"<<endl;
    exit(0);
  }
  ac::ReadPosesFromFile(filepath, poses);


  if(poses.size()==0){
    std::cerr<<"pose vector empty"<<std::endl;
    exit(0);
  }
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clouds;
  ac::ReadCloudsFromFile(directory_clouds, cloud_prefix, clouds, index_first_scan);
  if(poses.size()!=clouds.size()){
    std::cerr<<"Input data is of wrong size: clouds="<<clouds.size()<<", poses="<<poses.size()<<endl;
    exit(0);
  }
  else{
    cout<<"Found "<<poses.size()<<" poses and point clouds"<<endl;
  }

  ac::FilterCloudsByDistance(clouds, poses, min_distance);
  ac::SetScanLocations(clouds,poses);
  ac::ScanType::max_swell = max_swell;
  ac::ScanType::max_swell_dist = max_swell_distance;

  cout<<"output_dir: "<<output_dir<<endl;
  alignment_checker::AlignmentTester test(clouds, poses, ignore_idx, radius, type, downsample, rejection_ratio);

  //Logistics
  if(small_error){
    test.ReAlignScanFixedOffset(0.05, 0.005);
    test.PerformAndSaveTest(output_dir, output_file_name+"_small", dataset);
  }
  if(medium_error){
    test.ReAlignScanFixedOffset(0.1, 0.01);
    test.PerformAndSaveTest(output_dir, output_file_name+"_med", dataset);
  }
  if(large_error){
    test.ReAlignScanFixedOffset(0.2, 0.03);
    test.PerformAndSaveTest(output_dir, output_file_name+"_lar", dataset);
  }
  //ETH
/*
  if(small_error){
    test.ReAlignScanFixedOffset(0.05, 0.005);
    test.PerformAndSaveTest(output_dir, output_file_name+"_small", dataset);
  }
  if(medium_error){
    test.ReAlignScanFixedOffset(0.1, 0.01);
    test.PerformAndSaveTest(output_dir, output_file_name+"_med", dataset);
  }
  if(large_error){
    test.ReAlignScanFixedOffset(0.2, 0.03);
    test.PerformAndSaveTest(output_dir, output_file_name+"_lar", dataset);
  }
  */

  cout<<"finished"<<endl;


  return 0;
}



