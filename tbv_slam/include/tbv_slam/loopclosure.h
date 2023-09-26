#pragma once
#include <ros/ros.h>


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
#include <tf/transform_listener.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"

#include <time.h>
#include <fstream>
#include <cstdio>
#include "algorithm"
#include "map"


#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "tbv_slam/posegraph.h"
#include "cfear_radarodometry/types.h"
//#include "robust_mapping_custom_msgs/registration.h"
//#include "robust_mapping_custom_msgs/DetectLoop.h"

#include "ros/service.h"
//#include "robust_mapping_custom_msgs/n_registration.h"
#include "unordered_map"
#include "cfear_radarodometry/n_scan_normal.h"
#include "alignment_checker/AlignmentQuality.h"
#include "place_recognition_radar/RadarScancontext.h"
#include "tbv_slam/utils.h"
#include "alignment_checker/AlignmentQuality.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "alignment_checker/alignmentinterface.h"




/** \brief A ROS node which implements scan matching based on ndt
 * \author Daniel adolfsson
 *
 */
using std::string;
using std::cout;
using std::cerr;
using std::endl;
using namespace CFEAR_Radarodometry;
using namespace PlaceRecognitionRadar;
using namespace CorAlignment;


namespace tbv_slam{



class loopclosure{
public:


  struct BaseParameters
  {

    struct DerivedGTVicinityParameters // Simple Distance Based Closure
    {
      double min_d_travel_ = 40; // min distance traveled before revisit
      double max_d_travel_ = 4200; // min distance traveled before revisit
      double max_d_close_ = 15  ; // estimate of revisit needs to be within <max_d_registration_> meters form previous pose estimate
      bool GTVicinity_enabled = false;

      //double min_d_travel_ = 40; // min distance traveled before revisit
      //double max_d_travel_ = 4200; // min distance traveled before revisit
      //double max_d_close_ = 15  ; // estimate of revisit needs to be within <max_d_registration_> meters form previous pose estimate
    };
    struct DerivedMiniClosureParameters // Simple Distance Based Closure
    {
      double min_d_travel_ = 25; // Search range min
      double max_d_travel_ = 500; // Search range max
      double max_d_close_ = 15; // estimate of revisit needs to be within <max_d_registration_> meters form previous pose estimate
      bool MiniClosure_enabled = false;
    };
    struct DerivedSCCParameters //
    {
      bool SCClosure_enabled = true;
      size_t N_aggregate = 1 ;
      bool raw_scan_context = false;
      bool use_peaks = true;
      RSCManager::Parameters rsc_pars;

    };
    void GetParametersFromRos(ros::NodeHandle& nh);

    CorAlignment::PoseScan::Parameters posescan_par;
    DerivedSCCParameters DSCCP;
    DerivedGTVicinityParameters DGTVP;
    DerivedMiniClosureParameters DMCP;

    bool gt_loop = false;
    bool registration_disabled = false;

    // Verification disabld - for testing/training purposes
    bool verification_disabled = false;
    //Odometry verification
    bool verify_via_odometry = true;
    double odom_sigma_error = 0.03;
    bool visualize = true;
    bool debug_enabled = false;
    bool transl_guess = true;
    bool speedup = false;

    // Sampling for covariance estimation - used in loop closure scans registration
    bool use_covariance_sampling_in_loop_closure = false;

    // Verification classifier parameters
    double model_threshold = 0.8;
    bool load_trained_classifier = false;
    std::string model_training_file_load = "verification_training.txt";
    std::string model_training_file_save = "";
    bool all_candidates = true;   // Use all candidates (true) or only the one with greatest probability (false)
    std::vector<std::string> model_features = {"odom-bounds", "sc-sim", "alignment_quality"};

    //Debuging
    bool threading = true; //false;
    unsigned int idx_halt = 0;
    double Taddmin = 2, Taddmax = 0.1;
    std::string data_dir = "";
    std::string training_data_dir = "";
    unsigned int dataset_start_offset = 0;


    std::string namesToString()
    {
      return Join(GetStrMap(),true);
    }

    std::string valuesToString()
    {
      return Join(GetStrMap(),false);
    }

    const std::map<std::string,std::string> GetStrMap()const{
      std::map<std::string,std::string> map = {
        {"GTVicinity - min_d_travel_",std::to_string(DGTVP.min_d_travel_)},
        {"GTVicinity - max_d_travel_",std::to_string(DGTVP.max_d_travel_)},
        {"GTVicinity - max_d_close_",std::to_string(DGTVP.max_d_close_)},
        {"GTVicinity - GTVicinity_enabled",std::to_string(DGTVP.GTVicinity_enabled)},
        {"DerivedMiniClosure - min_d_travel_",std::to_string(DMCP.min_d_travel_)},
        {"DerivedMiniClosure - max_d_travel_",std::to_string(DMCP.max_d_close_)},
        {"DerivedMiniClosure - max_d_close_",std::to_string(DMCP.max_d_close_)},
        {"DerivedMiniClosure - MiniClosure_enabled",std::to_string(DMCP.MiniClosure_enabled)},
        {"registration_disabled",std::to_string(registration_disabled)},
        {"verification_disabled",std::to_string(verification_disabled)},
        {"verify_via_odometry",std::to_string(verify_via_odometry)},
        {"odom_sigma_error",std::to_string(odom_sigma_error)},
        {"idx_halt",std::to_string(idx_halt)},
        {"data_dir",data_dir},
        {"training_data_dir",training_data_dir},
        {"Scan Context - SCClosure_enabled",std::to_string(DSCCP.SCClosure_enabled)},
        {"Scan Context - N_aggregate",std::to_string(DSCCP.N_aggregate)},
        {"Scan Context - raw_scan_context",std::to_string(DSCCP.raw_scan_context)},
        {"Scan Context - use_peaks",std::to_string(DSCCP.use_peaks)},
        {"model_threshold",std::to_string(model_threshold)},
        {"model_training_file_load",model_training_file_load},
        {"all_candidates",std::to_string(all_candidates)},
      };
      auto rsc_pars_map =  DSCCP.rsc_pars.GetStrMap();
      map.insert(rsc_pars_map.begin(), rsc_pars_map.end());
      return map;
    }
    std::string ToString(){
      std::ostringstream ss;
      auto variables = GetStrMap();
      for(auto && variable : variables)
        ss << variable.first <<", "<<variable.second << endl;
      return ss.str();
    }

  };


  loopclosure(boost::shared_ptr<PoseGraph> graph, const BaseParameters& par ) : par_(par), nh_("~"), graph_(graph){
    //nh_.param<std::string>("registration_service_topic",service_topic, "registration");
    //nh_.param<int>("nr_subsequent_register_closure",nr_subsequent_register_closure_, 0);
    //reg_client = nh_.serviceClient<robust_mapping_custom_msgs::registration>(service_topic);
    if(par.load_trained_classifier) {
      sli_.LoadCoefficients(par.training_data_dir);
    }
    else {
      sli_.LoadData(par.training_data_dir);
      sli_.FitModels();
      sli_.SaveCoefficients(par.training_data_dir);
    }

    if(par_.threading)
      th = new std::thread(&loopclosure::LoopClosureThread, this);

    if(par_.model_training_file_load != ""){
      const std::string training_data_path = ros::package::getPath("tbv_slam") + "/model_parameters/" + par_.model_training_file_load;
      const std::string coef_data_path = ros::package::getPath("tbv_slam") + "/model_parameters/trained_loop_classifier.txt";
      if(par.load_trained_classifier) {
        verification_classifier_.LoadCoefficients(coef_data_path);
      }
      else {
        verification_classifier_.LoadData(training_data_path);
        verification_classifier_.fit();
        verification_classifier_.SaveCoefficients(coef_data_path);
      }
    }
  }

  bool ProcessFrame();

protected:

  void LoopClosureThread();

  virtual void SearchForConstraint(){}

  virtual bool SearchAndAddConstraint(){return false;} // return true as long as there is more to search, otherwise false

  double  DistanceTraveled(RadarScans::iterator& from, RadarScans::iterator& to);

  double  Distance(RadarScans::iterator& from, RadarScans::iterator& to);

  bool Register(pcl::PointCloud<pcl::PointXYZ>::Ptr src_local, pcl::PointCloud<pcl::PointXYZ>::Ptr target_local, const Eigen::Affine3d& Tfrom, const Eigen::Affine3d& Tto, Eigen::Affine3d& Tresponse, Eigen::Matrix<double, 6, 6> &reg_cov, double& quality);

  bool approximateCovarianceBySampling(n_scan_normal_reg &radar_reg, std::vector<CFEAR_Radarodometry::MapNormalPtr> &scans_vek, const std::vector<Eigen::Affine3d> &T_vek, Covariance &cov_sampled);

  bool RegisterLoopCandidate(const unsigned int from, const unsigned int to, Constraint3d& constraint);

  double VerifyLoopCandidate(Constraint3d& constraint);
  /*Given the estimated odometry, are these candidates reasonable loop constraints
   * Retuns the probability of being reasonable
   * <0.1 means " the poses are likely not nearby" and can be discarded p > 0.9 means there could be a loop
   * */

  void VerifyByOdometry(const unsigned int from, const unsigned int to, double& probability);

  void VerifyByAlignment(const unsigned int from, const Eigen::Affine3d& Tfrom_revised, const unsigned int to, const Eigen::Affine3d& Tfixed, std::map<string,double>& quality);

  double VerificationModel(std::map<string,double>& quality);

  Eigen::MatrixXd QualityToFeatures(std::map<string,double>& quality);

  void AddVerificationTrainingData(Constraint3d& constraint);

  void SaveVerificationTrainingData();

  void ApplyConstratins(std::vector<std::pair<double,Constraint3d>>& candidate_probabilities);

  /*!
   * Tguess is the guess in the global reference frame
   * Talign is response, e.g. the relative constraint T:from->to
   * reg_cov is given in the reference frame of Tfrom
   * from: moving
   * to:   fixed
   * for consistency, <from> is aquired later in time compared to <to>,
   */

  bool Register(const unsigned int from, const unsigned int to, const Eigen::Affine3d& Tfrom, const Eigen::Affine3d& Tto, Eigen::Affine3d& Talign, Eigen::Matrix<double, 6, 6> &reg_cov);

  //void CreateSrvMsg(const std::vector<unsigned int> &from, const std::vector<unsigned int> &to, robust_mapping_custom_msgs::registration &srv);

  BaseParameters par_; // Parameters

  size_t prev_size_ = 0;
  RadarScans::iterator itr_current;
  std::map<std::pair<unsigned int,unsigned int>,bool> pair_attempted_; // List of index combinations for which have already been attempted
  std::map<unsigned int,bool> origin_attempted_;
  std::string data_folder;
  CorAlignment::ScanLearningInterface sli_;
  CorAlignment::LogisticRegression verification_classifier_;

  std::thread *th;
  boost::shared_ptr<PoseGraph> graph_;
  ros::ServiceClient reg_client;
  ros::NodeHandle nh_;
  std::string service_topic = "registration";
  int nr_subsequent_register_closure_ = 0;
  bool scan_iterator_initialized_ = false;  // delay the initialization of the iterator until the first scan is in the graph
};

class GTVicinityClosure : public loopclosure{
public:

  GTVicinityClosure(boost::shared_ptr<PoseGraph> graph, const BaseParameters& par ) : loopclosure(graph,par),
    min_d_travel_(par.DGTVP.min_d_travel_),
    max_d_travel_(par.DGTVP.max_d_travel_),
    max_d_close_(par.DGTVP.max_d_close_),
    GTVicinity_enabled_(par.DGTVP.GTVicinity_enabled){
    cout << " GTVicinityClosure " << endl;
  }

  bool SearchAndAddConstraint();

  double min_d_travel_; // Search range min
  double max_d_travel_; // Search range max
  double max_d_close_; // estimate of revis
  bool GTVicinity_enabled_;

};

class MiniClosure : public loopclosure{
public:


  MiniClosure(boost::shared_ptr<PoseGraph> graph, const BaseParameters& par ) : loopclosure(graph,par),
    min_d_travel_(par.DMCP.min_d_travel_),
    max_d_travel_(par.DMCP.max_d_travel_),
    max_d_close_(par.DMCP.max_d_close_),
    MiniClosure_enabled(par.DMCP.MiniClosure_enabled){
    cout << " MiniClosure " << endl;
  }

  bool SearchAndAddConstraint();
private:

  double min_d_travel_; // Search range min
  double max_d_travel_; // Search range max
  double max_d_close_; // estimate of revisit needs to be within <max_d_registration_> meters form previous pose estimate
  bool MiniClosure_enabled;

};


class ScanContextClosure : public loopclosure{
public:
  ScanContextClosure(boost::shared_ptr<PoseGraph> graph, const BaseParameters& par ) : loopclosure(graph, par), rsc_pars(par.DSCCP.rsc_pars), rsc_(rsc_pars){
    reader_ = std::unique_ptr<RadarReader>(std::make_unique<PNGReaderInterface>(par.data_dir));
    Todom_ = Eigen::Affine3d::Identity();
    //client_ = nh_.serviceClient<robust_mapping_custom_msgs::DetectLoop>(loop_client_topic_);
  }
  bool SearchAndAddConstraint();

  void ProcessFrame();

  pcl::PointCloud<pcl::PointXYZI>::Ptr ScansToLocalMap(RadarScans::iterator& itr, const Eigen::Affine3d& T);

  void CreateContext(RadarScans::iterator& itr, Eigen::Affine3d& Todom);

  // use translation uses
  void RegisterLoopCandidates();

  inline Eigen::Matrix<double,6,6> Score2Cov(double score);

protected:

  RSCManager::Parameters rsc_pars;
  RSCManager rsc_;
  std::unique_ptr<RadarReader> reader_;
  Eigen::Affine3d Todom_;

  double p_min_, rel_dist_sigma_;
  std::map<std::pair<int,int>, bool> places_recognized, places_registered;
  ros::ServiceClient client_;
  std::string loop_client_topic_;
};


class DebugConstraints : public loopclosure{
public:
  DebugConstraints(boost::shared_ptr<PoseGraph> graph, const BaseParameters& par ) : loopclosure(graph, par), rsc_pars(par.DSCCP.rsc_pars), rsc_(rsc_pars){

  }
  bool SearchAndAddConstraint();

protected:

  RSCManager::Parameters rsc_pars;
  RSCManager rsc_;

  ros::ServiceClient client_;
  std::string loop_client_topic_;
};

  template<typename T> std::vector<double> linspace(T start_in, T end_in, int num_in); // useful equivalent of Matlab's linspace

}

