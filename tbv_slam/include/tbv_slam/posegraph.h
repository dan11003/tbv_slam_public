#pragma once

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"


#include "stdio.h"
#include "cfear_radarodometry/types.h"
#include "tbv_slam/pose_graph_optimization.h"
#include "mutex"
#include "boost/shared_ptr.hpp"
#include "thread"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_broadcaster.h"
#include "eigen_conversions/eigen_msg.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
#include "visualization_msgs/MarkerArray.h"

#include <boost/serialization/base_object.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "boost/serialization/serialization.hpp"
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include "boost/serialization/shared_ptr.hpp"
//#include "ndt_generic/eigen_utils.h"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/map.hpp"
#include "pcl/common/io.h"
#include "pcl/filters/random_sample.h"
#include "tuple"
#include "iterator"
#include "nav_msgs/Path.h"
#include "zip.h"
#include "tbv_slam/safe_queue.h"
#include "cfear_radarodometry/eval_trajectory.h"
#include "place_recognition_radar/EvaluationManager.h"
#include "tbv_slam/ceresoptimizer.h"



namespace tbv_slam{
using std::cout;
using std::endl;
using namespace CFEAR_Radarodometry;
using namespace PlaceRecognitionRadar;
//typedef std::map<int, std::less<int>,pcl::PointCloud<pcl::PointXYZ>::Ptr >MapOfClouds;
class PoseGraph;
typedef boost::shared_ptr<PoseGraph> PoseGraphPtr;

class PoseGraph
{
public:
  class Parameters {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:


    Parameters() {}

    bool disable_optimization = false; //mostly for debug purposes
    double Toptimize = 1.0 ;
    std::string est_output_dir = "", gt_output_dir = "";
    EvaluationManager::Parameters eval_manager_par;

    tbv_slam::PGOptimizationParameters opt_pars;
    bool online;


    void GetParametersFromRos( ros::NodeHandle& param_nh){
      // param_nh.param<std::string>("input_points_topic", input_points_topic, "/Navtech/Filtered");
    }

    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "OdometryKeyframeFuser::Parameters"<<endl;
      stringStream << "disable_optimization, "<<disable_optimization<<endl;
      stringStream << "est_output_dir, "<<est_output_dir<<endl;
      stringStream << "gt_output_dir, "<<gt_output_dir<<endl;

      return stringStream.str();
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      ar & disable_optimization;
      ar & Toptimize;
      ar & est_output_dir;
      ar & gt_output_dir;
    }
  };

  PoseGraph(PoseGraph::Parameters& par);

  //void AddNode(const Eigen::Affine3d &Tdiff, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,Eigen::Matrix<double,6,6> &Cov, int id, int id_prev);

  /******** Management ********/
  void AddSimpleGraphUnsafe(simple_graph& sg); //Thread safe

  void AddNodeThSafe(std::pair<RadarScan, std::vector<Constraint3d>>& node_constraint); //Thread safe



  void AddConstraintThSafe(const Constraint3d& constraint); //Thread safe

  void ForceOptimize(); // Instead of OptimizeThread

  bool InputQueueEmpty(){return queue_new_nodes_.empty() && queue_new_loopconstraints_.empty();}



  /****** Nodes *****/

  RadarScanHandler& GetPoses(){return nodes_;}

  Eigen::Affine3d GetPose(unsigned int id){return nodes_.GetScan(id).GetPose();}

  bool NodeExists(unsigned int id){return nodes_.NodeExists(id);}

  size_t size(){ return nodes_.size();}

  RadarScan& operator[](int index){return nodes_.Get(index)->second;}




  /********** Constraints ***************/

  ConstraintsHandler& GetConstraints(){return constraints_;}

  ConstraintsHandler GetConstraintsCopy(){return constraints_;}

  bool ConstraintExist(unsigned int id1, unsigned int id2);

  /*********** METRICS *******************/

  double TraveledDistance(unsigned int id1, unsigned int id2);

  double EuclidianDistance(unsigned int id1, unsigned int id2);

  void UpdateStatistics(const std::pair<unsigned int,unsigned int> & guess, const Eigen::Affine3d& Tguess, const std::map<std::string,double>& quality, int guess_nr);



  /*********** Input / Output *************/

  std::string ToString();

  void SaveGraphString(const std::string& path);

  static void SaveGraph(const std::string& path, PoseGraphPtr &graph);

  static bool LoadGraph(const std::string& path, PoseGraphPtr &graph);

  static bool LoadSimpleGraph(const std::string& path, PoseGraphPtr &graph, PoseGraph::Parameters& par);

  void OutputGraph();

  Eigen::Matrix<double, 6, 6> GetCovariance(unsigned int from, unsigned int to);

  void Align();

  std::mutex m_graph;

  PoseGraph::Parameters par_;

  bool optimize = true; //false;

  PlaceRecognitionRadar::EvaluationManager eval_manager; // for evaluation only

protected:

  PoseGraph() : eval_manager(EvaluationManager::Parameters()){}

  void AddNodeThread();

  void AddConstraintThread();

  void OptimizerThread();




  //std::vector<std::pair<RadarScan,std::vector<Constraint3d>>> queue_new_nodes_;
  //std::vector<Constraint3d> queue_new_loopconstraints_;
  typedef std::pair<RadarScan,std::vector<Constraint3d>> qnode;
  SafeQueue<qnode> queue_new_nodes_;
  SafeQueue<Constraint3d> queue_new_loopconstraints_;
  RadarScanHandler nodes_; // sensor pose in world
  ConstraintsHandler constraints_; // hash to all constraints

  std::vector<std::thread*>  thread_vec;
  ros::NodeHandle nh_;

  bool keep_running = true, optimized = false;

  double traveled_ = 0;




  friend class PoseGraphVis;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    //ar & queue_new_nodes_;
    ar & nodes_;
    ar & constraints_;
    ar & keep_running;
    ar & par_;
  }

};


class PoseGraphVis{

public:
  class Parameters{
  public:

    Parameters() {}

    void GetParametersFromRos( ros::NodeHandle& param_nh);

    std::string optimized_map_topic_ = "map_optimized";
    std::string constraints_loop_topic_ = "loop_constraints";
    std::string constraints_odom_topic_ = "odom_constraints";
    std::string constraints_candidate_topic_ = "candidate_constraints";
    std::string constraints_gt_loop_topic_ = "gt_loop_constraints";
    std::string gt_path_topic_ = "gt_path";

    std::string world_frame = "world";

    int Tupd = 2.0;
    int skip_frames = 1;
    bool disable_odom_constraints_ = false;
    bool visualization_enabled_  = true;
    bool threading = true;
  };

  PoseGraphVis(PoseGraphPtr graph, const Parameters& par = PoseGraphVis::Parameters());

  void ProcessFrame();

  static void pubTFForPose(const std::vector<Eigen::Affine3d>& pose, const std::vector<std::string>& name, const ros::Time& t);



private:

  std_msgs::ColorRGBA red, green, blue;

  void PlotAll(RadarScanHandler& scans, ConstraintsHandler& constraints_);

  void VisualizeThread();

  visualization_msgs::Marker CreateDefaultMarker(const std::string &ns="");

  void PublishConstraints( ConstraintsHandler& constraints, RadarScanHandler& scans);

  visualization_msgs::MarkerArray ConstriantsToMarker(ConstraintType ct, ConstraintsHandler& ch,  RadarScanHandler& sh, const std_msgs::ColorRGBA& color, bool absolute, bool ground_truth, const double& z_offset = 0);

  PoseGraphVis::Parameters par_;



  PoseGraphPtr graph_;
  std::thread *th;
  bool despawn_thread_ = false;
  bool force_update_ = false;


  ros::NodeHandle nh_;
  ros::Publisher vis_clouds, vis_gt_clouds;
  ros::Publisher pub_odom_constraint, pub_loop_constraint, pub_candidate_constraint, pub_gt_path, pub_odom_path, pub_slam_path, pub_text_constraint;
  tf::TransformBroadcaster Tb;
};

void PublishMarkerArray(const std::string& topic, visualization_msgs::MarkerArray& msg);

}
