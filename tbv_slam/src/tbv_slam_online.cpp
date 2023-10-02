
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tbv_slam/OptimizationParamsConfig.h>

#include <time.h>
#include <fstream>
#include <cstdio>

#include "cfear_radarodometry/types.h"
#include "tbv_slam/posegraph.h"
#include "tbv_slam/loopclosure.h"
#include "boost/program_options.hpp"
#include "tbv_slam/tbv_slam.h"

#include "cfear_radarodometry/radar_driver.h"
#include "cfear_radarodometry/odometrykeyframefuser.h"
#include "cfear_radarodometry/eval_trajectory.h"
#include "alignment_checker/alignmentinterface.h"

#include "boost/foreach.hpp"
#include "rosbag/view.h"
#include "rosbag/bag.h"

#define foreach BOOST_FOREACH

/** \brief A ROS node which implements robust radar slam
 * \author Daniel adolfsson
 *
 */


using std::string;
using std::cout;
using std::cerr;
using std::endl;
namespace po = boost::program_options;

using namespace tbv_slam;

typedef struct cfear_eval_parameters_{
  std::string bag_file_path ="";
  bool save_pcds = false;
  bool save_radar_img = false;
  std::string radar_dir ="";
}cfear_eval_parameters;

typedef struct training_parameters_{
  std::string training_dir ="";
  bool save_ROC = false;
  bool disable_training = false;
}training_parameters;

class tbv_eval_parameters
{
public:
  tbv_eval_parameters() {}
  std::string input_directory;
  std::string dataset;
  std::string sequence;
  std::string simple_graph_path;
  std::string full_graph_path;
  std::string eval_output_dir;
  bool visualize;
  bool wait_for_key;
  bool save_graph;
  bool load_graph;
  bool debug_optimizer;
  std::string experiment_name;
  std::string method;

  std::string ToString(){
    std::ostringstream stringStream;
    //stringStream << "OdometryKeyframeFuser::Parameters"<<endl;
    stringStream << "input_directory, "<<input_directory<<endl;
    stringStream << "dataset, "<<dataset<<endl;
    stringStream << "sequence, "<<sequence<<endl;
    stringStream << "simple_graph path, "<<simple_graph_path<<endl;
    stringStream << "eval_output dir, "<<eval_output_dir<<endl;
    stringStream << "visualize, "<<std::boolalpha<<visualize<<endl;
    stringStream << "experiment name, "<<experiment_name<<endl;
    stringStream << "method, "<<method<<endl;

    return stringStream.str();
  }

};

class radarReader
{

private:

  ros::NodeHandle nh_;
  ros::Publisher pub_odom;
  EvalTrajectory eval;
  std::string output_dir;
  radarDriver driver;
  OdometryKeyframeFuser fuser;
  Eigen::Affine3d Toffset = Eigen::Affine3d::Identity();

  ScanLearningInterface scan_learner;
  training_parameters training_pars;

public:

  radarReader(const OdometryKeyframeFuser::Parameters& odom_pars,
              const radarDriver::Parameters& rad_pars,
              const EvalTrajectory::Parameters& eval_par,
              const cfear_eval_parameters& p,
              const training_parameters& training_par,
              const PoseGraphPtr graph) : nh_("~"), driver(rad_pars,true), fuser(odom_pars, true), eval(eval_par,true),output_dir(eval_par.est_output_dir), training_pars(training_par){

    pub_odom = nh_.advertise<nav_msgs::Odometry>("/gt", 1000);
    cout<<"Loading bag from: "<<p.bag_file_path<<endl;
    rosbag::Bag bag;
    bag.open(p.bag_file_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {"/Navtech/Polar","/gt","/your_polar_image"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int frame = 0;
    //std::map<ros::Time, std::pair<RadarScan, std::vector<Constraint3d>>> node_map;
    std::vector<std::pair<ros::Time, std::pair<RadarScan, std::vector<Constraint3d>>>> nodes;

    foreach(rosbag::MessageInstance const m, view)
    {

      if(!ros::ok())
        break;

      nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
      if (odom_msg != NULL){
        nav_msgs::Odometry msg_odom = *odom_msg;
        poseStamped stamped_gt_pose(Eigen::Affine3d::Identity(), Covariance::Identity(), odom_msg->header.stamp);
        tf::poseMsgToEigen(msg_odom.pose.pose, stamped_gt_pose.pose);
        //stamped_gt_pose.pose = stamped_gt_pose.pose;//transform into sensor frame
        Eigen::Matrix4d m = stamped_gt_pose.pose.matrix();
        m(0,2) = 0; m(2,0) = 0; m(2,1) = 0; m(1,2) = 0; m(2,2) = 1; // 3d -> 2d
        stamped_gt_pose.pose = Eigen::Affine3d(m);
        static Eigen::Affine3d Tfirst_i = stamped_gt_pose.pose.inverse();
        stamped_gt_pose.pose = Tfirst_i*stamped_gt_pose.pose;
        eval.CallbackGTEigen(stamped_gt_pose);

        msg_odom.header.stamp = ros::Time::now();
        msg_odom.header.frame_id = "world";
        pub_odom.publish(msg_odom);
        continue;
      }

      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      if(image_msg != NULL) {
        ros::Time tinit = ros::Time::now();
        //if(frame==0)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, cloud_filtered_peaks;
        driver.CallbackOffline(image_msg, cloud_filtered, cloud_filtered_peaks);
        CFEAR_Radarodometry::timing.Document("Filtered points",cloud_filtered->size());
        Eigen::Affine3d Tcurrent;
        Covariance cov_current;
        
        ros::Time t;
        pcl_conversions::fromPCL(cloud_filtered->header.stamp, t);

        ros::Time t1 = ros::Time::now();
        fuser.pointcloudCallback(cloud_filtered, cloud_filtered_peaks, Tcurrent, t, cov_current);
        ros::Time t2 = ros::Time::now();
        CFEAR_Radarodometry::timing.Document("Odometry",CFEAR_Radarodometry::ToMs(t2-t1));
        if(fuser.updated && p.save_radar_img){
          const std::string path = p.radar_dir + std::to_string(t.toNSec())+".png";
          cv::imwrite(path, driver.cv_polar_image->image);
        }
        if(fuser.updated) {
          auto node = fuser.GetLastNode();
          nodes.emplace_back(std::make_pair(image_msg->header.stamp, node));
          /*
          if (image_msg->header.stamp != ros::Time(0,0)) {
            node_map[image_msg->header.stamp] = node;
          } else { // No time stamps available, so no timesync to GT possible
            graph->AddNodeThSafe(node);
          }
          */
        }
        
        // Load current_scan with data from fuser.scan_, then add current_scan to scan_learner
        if(!training_par.disable_training){
          ScanLearningInterface::s_scan current_scan = {fuser.scan_.GetPose(), fuser.scan_.cloud_nopeaks_, fuser.scan_.cloud_peaks_, fuser.scan_.cloud_normal_};
          scan_learner.AddTrainingData(current_scan);
        }
        eval.CallbackESTEigen(poseStamped(Tcurrent, cov_current, t));
        ros::Time tnow = ros::Time::now();
        ros::Duration d = ros::Duration(tnow-tinit);
        static ros::Duration tot(0);
        tot +=d;
        //usleep(100*1000);


        cout<<"Frame: "<<frame<<", dur: "<<d<<", avg: "<<++frame/tot.toSec()<<endl;
      }
      //for (auto pose_it = node_map.begin(); pose_it != node_map.end();) {
      for (auto pose_it = nodes.begin(); pose_it != nodes.end();) {
        poseStamped interpolated;
        if(eval.Interpolate(pose_it->first, interpolated)) {
          pose_it->second.first.Tgt = interpolated.pose;
          pose_it->second.first.has_Tgt_ = true;
          graph->AddNodeThSafe(pose_it->second);
          //node_map.erase(pose_it++);
          pose_it = nodes.erase(pose_it);
        }
        //else if (node_map.size() > 1) {
        else if (nodes.size() > 1) {// happens before GT is available or if GT is not available
          graph->AddNodeThSafe(pose_it->second);
          //node_map.erase(pose_it++);
          pose_it = nodes.erase(pose_it);
        }
        else
          ++pose_it;
      }
    }
    cout<<fuser.GetStatus()<<endl;
    bag.close();
    CFEAR_Radarodometry::timing.PresentStatistics();
    std::ofstream statistics_file;
    statistics_file.open (output_dir + "/time_statistics.txt");
    statistics_file << CFEAR_Radarodometry::timing.GetStatistics();
    statistics_file.close();
    return;
  }

  void Save(){
    eval.Save(); // must be called before ground
    poseStampedVector& vek(eval.GetGtVek());
    fuser.AddGroundTruth(vek);

    fuser.SaveGraph(output_dir+"simple_graph.sgh");

    // Save training data
    if(!training_pars.disable_training) {
      scan_learner.FitModels();
      scan_learner.SaveData(training_pars.training_dir);
      scan_learner.SaveCoefficients(training_pars.training_dir);
      if(training_pars.save_ROC)
        scan_learner.SaveROCCurves(training_pars.training_dir);
    }
    return;
  }

  ~radarReader(){
    Save();
    std::cout << "exited radarReader succesfully" << std::endl;
    return;
  }
  size_t GetSize(){return eval.GetSize();}
};

/*

void ReadOptions(const int argc, char**argv,  eval_parameters& p, ){

  po::options_description desc{"Options"};
  desc.add_options()
      ("help,h", "Help screen");
      

  po::variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);
  notify(vm);

  if (vm.count("help"))
    std::cout << desc << '\n';
  

}
*/


void ReadOptions(const int argc, char**argv, OdometryKeyframeFuser::Parameters& par, radarDriver::Parameters& rad_par, CFEAR_Radarodometry::EvalTrajectory::Parameters& eval_par, cfear_eval_parameters& p, training_parameters& training_par,tbv_eval_parameters& tbv_eval_par, PoseGraph::Parameters& pose_graph_par, TBVSLAM::Parameters& slam_pars, PoseGraphVis::Parameters& pose_vis_par){

  po::options_description desc{"Options"};
  desc.add_options()
      ("help,h", "Help screen")
      ("res", po::value<double>()->default_value(3.5), "res")
      ("range-res", po::value<double>()->default_value(0.0438), "range resolution")
      ("min_distance", po::value<double>()->default_value(2.5), "min sensor distance")
      ("max_distance", po::value<double>()->default_value(200), "mib sensor distance ")
      ("submap_scan_size", po::value<int>()->default_value(3), "submap_scan_size")
      ("weight_intensity", po::value<bool>()->default_value(true),"weight_intensity")
      ("k_strongest", po::value<int>()->default_value(12), "kstrongest points filtering")
      ("job_nr", po::value<int>()->default_value(-1), "jobnr")
      ("registered_min_keyframe_dist", po::value<double>()->default_value(1.5), "registered_min_keyframe_dist")
      ("z-min", po::value<double>()->default_value(65), "zmin intensity, expected noise level")
      ("radar_ccw", po::value<bool>()->default_value(false),"radar_ccw")
      ("soft_constraint", po::value<bool>()->default_value(false),"soft_constraint")
      ("savepcd", "save_pcd_files")
      ("disable_compensate", po::value<bool>()->default_value(false),"disable_compensate")
      ("cost_type", po::value<std::string>()->default_value("P2L"), "P2L")
      ("loss_type", po::value<std::string>()->default_value("Huber"), "robust loss function eg. Huber Caunchy, None")
      ("loss_limit", po::value<double>()->default_value(0.1), "loss limit")
      ("covar_scale", po::value<double>()->default_value(1), "covar scale")// Please fix combined parameter
      ("covar_sampling", po::value<bool>()->default_value(false), "Covar. by cost sampling")
      ("covar_sample_save", po::value<bool>()->default_value(false), "Dump cost samples to a file")
      ("covar_sample_dir", po::value<std::string>()->default_value("/tmp/cfear_out"), "Where to save the cost samples")
      ("covar_XY_sample_range", po::value<double>()->default_value(0.4), "Sampling range on the x and y axes")
      ("covar_yaw_sample_range", po::value<double>()->default_value(0.0043625), "Sampling range on the yaw axis")
      ("covar_samples_per_axis", po::value<int>()->default_value(3), "Num. of samples per axis for covar. sampling")
      ("covar_sampling_scale", po::value<double>()->default_value(4), "Sampled covar. scale")
      ("regularization", po::value<double>()->default_value(1), "regularization")
      ("est_directory", po::value<std::string>()->default_value(""), "output folder of estimated trajectory")
      ("gt_directory", po::value<std::string>()->default_value(""), "output folder of ground truth trajectory")
      ("sequence", po::value<std::string>()->default_value("2019-01-10-12-32-52-radar-oxford-10k"), "sequence contrained in \"bagfile\" to evaluate e.g. 2019-01-10-12-32-52-radar-oxford-10k")
      ("dataset", po::value<std::string>()->default_value("oxford"), "name of dataset, take special actions depending on radar file format etc")
      ("filter-type", po::value<std::string>()->default_value("kstrong"), "filter type")
      ("cfear_method_name", po::value<std::string>()->default_value("method"), "method name")
      ("weight_option", po::value<int>()->default_value(0), "how to weight residuals")
      ("false-alarm-rate", po::value<float>()->default_value(0.01), "CA-CFAR false alarm rate")
      ("nb-guard-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
      ("nb-window-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
      ("store_graph", po::value<bool>()->default_value(false),"store_graph")
      ("save_radar_img", po::value<bool>()->default_value(false),"save_radar_img")
      ("bag_path", po::value<std::string>()->default_value("/home/daniel/rosbag/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar/2019-01-10-12-32-52-radar-oxford-10k.bag"), "bag file to open")
      ("training_directory", po::value<std::string>()->default_value(""), "output folder of training data")
      ("disable_training", po::value<bool>()->default_value(false), "disable generation of training data")
      ("radar_directory", po::value<std::string>()->default_value(""), "output folder of radar images")
      ("save_ROC", po::value<bool>()->default_value(false), "save ROC-curves")
      ("visualize", po::value<bool>()->default_value(false),"visualize")
      ("miniloop-enabled", po::value<bool>()->default_value(false),"short distance miniloop closure")
      ("wait_for_key", po::value<bool>()->default_value(false),"wait for keypress")
      ("input_directory", po::value<std::string>()->default_value(""), "directory of odometry sequences")
      ("output_directory", po::value<std::string>()->default_value(""), "eval_output_dir")
      ("experiment_name", po::value<std::string>()->default_value(""), "eval_output_dir")
      ("tbv_method_name", po::value<std::string>()->default_value("TBV SLAM-8"), "tbv method name")
      ("plot_descriptor", po::value<bool>()->default_value(false), "plot descriptor")
      ("sc_th", po::value<double>()->default_value(0.8), "plot descriptor")
      ("NUM_CANDIDATES_FROM_TREE", po::value<int>()->default_value(10), "NUM_CANDIDATES_FROM_TREE")
      ("PC_NUM_RING", po::value<int>()->default_value(40), "PC_NUM_RING")
      ("PC_NUM_SECTOR", po::value<int>()->default_value(120), "PC_NUM_SECTORS")
      ("PC_MAX_RADIUS", po::value<double>()->default_value(80), "PC_MAX_RADIUS")
      ("no_point", po::value<double>()->default_value(0.0), "value in descriptor for NO_POINT")
      ("desc_function", po::value<std::string>()->default_value("sum"), "descriptor generation function")
      ("desc_divider", po::value<double>()->default_value(1000.0), "descriptor divider value")
      ("N_CANDIDATES", po::value<int>()->default_value(3), "N_CANDIDATES")
      ("N_aggregate", po::value<int>()->default_value(1), "N_aggregate")
      ("verify_via_odometry", po::value<bool>()->default_value(true), "verify_via_odometry")
      ("use_covariance_sampling_in_loop_closure", po::value<bool>()->default_value(false), "use covariance sampling in the loop closure registration")
      ("replace_cov_by_identity", po::value<bool>()->default_value(true), "use covariance sampling in the loop closure registration")
      ("raw_radar_scan", po::value<bool>()->default_value(false), "raw_radar_scan descriptor")
      ("odom_sigma_error", po::value<double>()->default_value(0.05), "odom_sigma_error, upper bound of odometry error")
      ("idx_halt", po::value<int>()->default_value(0), "at this point, start debuging")
      ("dataset_offset", po::value<int>()->default_value(0), "at this point, start debuging")
      ("registration_disabled", po::value<bool>()->default_value(false), "disable registraiton")
      ("verification_disabled", po::value<bool>()->default_value(false), "verification_disabled for the purpose of viewing ALL constaints in rviz")
      ("odometry_coupled_closure", po::value<bool>()->default_value(true),"odometry_coupled_closure")
      ("augment_sc", po::value<bool>()->default_value(true),"save_radar_img")
      ("simple_graph_name", po::value<std::string>()->default_value("simple_graph.sgh"), "graph path")
      ("full_graph_name", po::value<std::string>()->default_value("full_graph.pgh"), "Full graph path")
      ("save_graph", po::value<bool>()->default_value(false), "Save after loop closure")
      ("load_graph", po::value<bool>()->default_value(false), "Load the full graph")
      ("use_default_alignment_data", po::value<bool>()->default_value(true), "Load the full graph")
      ("gt_loop", po::value<bool>()->default_value(false), "ground truth loop")
      ("optimization_disabled", po::value<bool>()->default_value(false), "ground truth loop")
      ("debug_enabled", po::value<bool>()->default_value(false), "Do not run the loop closure")
      ("speedup", po::value<bool>()->default_value(false), "ommit verification of not needed")
      ("debug_optimizer", po::value<bool>()->default_value(false), "Do not run the loop closure")
      ("all_candidates", po::value<bool>()->default_value(false), "Use all candidates (true), or only use best probability candidate (false)")
      ("load_trained_classifier", po::value<bool>()->default_value(false), "if true, do not retrain classifier but use stored coefficients (located in /model_parameters)")
      ("model_training_file_load", po::value<std::string>()->default_value("tbv_model_8.txt"), "File name for verificaiton classifier training data (located in /model_parameters)")
      ("model_training_file_save", po::value<std::string>()->default_value(""), "File name for verificaiton classifier training data (saved in /model_parameters)")
      ("model_threshold", po::value<double>()->default_value(0.9), "Threshold for candidate verification classifier")
      ("model_features", po::value<std::vector<std::string>>()->multitoken(), "Quality features to use in model classifier");


  po::variables_map vm;
  store(parse_command_line(argc, argv, desc), vm);
  notify(vm);

  if (vm.count("help"))
    std::cout << desc << '\n';


  tbv_eval_par.input_directory = vm["input_directory"].as<std::string>();
  tbv_eval_par.dataset = vm["dataset"].as<std::string>();
  pose_graph_par.eval_manager_par.dataset = vm["dataset"].as<std::string>();
  pose_graph_par.eval_manager_par.sequence = vm["sequence"].as<std::string>();
  pose_graph_par.online = true;
  tbv_eval_par.sequence = vm["sequence"].as<std::string>();
  tbv_eval_par.eval_output_dir = vm["output_directory"].as<std::string>();
  tbv_eval_par.simple_graph_path = tbv_eval_par.input_directory + "/" + tbv_eval_par.dataset + "/" + tbv_eval_par.sequence + "/est/" +  vm["simple_graph_name"].as<std::string>();
  tbv_eval_par.full_graph_path = tbv_eval_par.input_directory + "/" + tbv_eval_par.dataset + "/" + tbv_eval_par.sequence + "/est/" +  vm["full_graph_name"].as<std::string>();
  tbv_eval_par.save_graph = vm["save_graph"].as<bool>();
  tbv_eval_par.load_graph = vm["load_graph"].as<bool>();

  slam_pars.loop_pars.gt_loop = vm["gt_loop"].as<bool>();
  slam_pars.loop_pars.threading = true;
  pose_graph_par.disable_optimization = vm["optimization_disabled"].as<bool>();
  tbv_eval_par.debug_optimizer = slam_pars.loop_pars.debug_enabled = vm["debug_enabled"].as<bool>();

  slam_pars.loop_pars.data_dir = tbv_eval_par.input_directory + "/" + tbv_eval_par.dataset + "/" + tbv_eval_par.sequence + "/radar" ;

  if( vm["use_default_alignment_data"].as<bool>() ){
    slam_pars.loop_pars.training_data_dir = ros::package::getPath("tbv_slam") + "/model_parameters/";
  }
  else{
    slam_pars.loop_pars.training_data_dir = tbv_eval_par.input_directory + "/" + tbv_eval_par.dataset + "/" + tbv_eval_par.sequence + "/training" ;
  }


  slam_pars.loop_pars.visualize = tbv_eval_par.visualize = pose_vis_par.visualization_enabled_ = vm["visualize"].as<bool>();
  tbv_eval_par.wait_for_key = vm["wait_for_key"].as<bool>();
  tbv_eval_par.experiment_name = vm["experiment_name"].as<std::string>();
  tbv_eval_par.method = vm["tbv_method_name"].as<std::string>();
  slam_pars.loop_pars.DMCP.MiniClosure_enabled = vm["miniloop-enabled"].as<bool>();
  slam_pars.loop_pars.DSCCP.rsc_pars.desc_plot = vm["plot_descriptor"].as<bool>();
  slam_pars.loop_pars.DSCCP.rsc_pars.NUM_CANDIDATES_FROM_TREE = vm["NUM_CANDIDATES_FROM_TREE"].as<int>();
  slam_pars.loop_pars.DSCCP.rsc_pars.PC_NUM_RING = vm["PC_NUM_RING"].as<int>();
  slam_pars.loop_pars.DSCCP.rsc_pars.PC_NUM_SECTORS = vm["PC_NUM_SECTOR"].as<int>();
  slam_pars.loop_pars.DSCCP.rsc_pars.PC_MAX_RADIUS = vm["PC_MAX_RADIUS"].as<double>();
  slam_pars.loop_pars.DSCCP.rsc_pars.no_point = vm["no_point"].as<double>();
  slam_pars.loop_pars.DSCCP.rsc_pars.desc_function = vm["desc_function"].as<std::string>();
  slam_pars.loop_pars.DSCCP.rsc_pars.desc_divider = vm["desc_divider"].as<double>();
  slam_pars.loop_pars.DSCCP.rsc_pars.SC_DIST_THRES = vm["sc_th"].as<double>();
  slam_pars.loop_pars.DSCCP.rsc_pars.N_CANDIDATES = vm["N_CANDIDATES"].as<int>();
  slam_pars.loop_pars.verify_via_odometry = vm["verify_via_odometry"].as<bool>();
  slam_pars.loop_pars.DSCCP.raw_scan_context = vm["raw_radar_scan"].as<bool>();
  slam_pars.loop_pars.odom_sigma_error = vm["odom_sigma_error"].as<double>();
  slam_pars.loop_pars.idx_halt = vm["idx_halt"].as<int>();
  slam_pars.loop_pars.dataset_start_offset= vm["dataset_offset"].as<int>();
  slam_pars.loop_pars.verification_disabled = vm["verification_disabled"].as<bool>();
  slam_pars.loop_pars.registration_disabled= vm["registration_disabled"].as<bool>();
  slam_pars.loop_pars.DSCCP.N_aggregate = vm["N_aggregate"].as<int>();
  slam_pars.loop_pars.DSCCP.rsc_pars.odometry_coupled_closure = vm["odometry_coupled_closure"].as<bool>();
  slam_pars.loop_pars.DSCCP.rsc_pars.augment_sc = vm["augment_sc"].as<bool>();
  slam_pars.loop_pars.speedup = vm["speedup"].as<bool>();
  slam_pars.loop_pars.use_covariance_sampling_in_loop_closure = vm["use_covariance_sampling_in_loop_closure"].as<bool>();
  pose_graph_par.opt_pars.ceres_pars.scaling_pars.replace_cov_by_identity = vm["replace_cov_by_identity"].as<bool>();


  slam_pars.loop_pars.load_trained_classifier = vm["load_trained_classifier"].as<bool>();
  slam_pars.loop_pars.model_training_file_load = vm["model_training_file_load"].as<std::string>();
  slam_pars.loop_pars.model_training_file_save = vm["model_training_file_save"].as<std::string>();
  slam_pars.loop_pars.all_candidates = vm["all_candidates"].as<bool>();
  slam_pars.loop_pars.model_threshold = vm["model_threshold"].as<double>();
  slam_pars.loop_pars.model_features = vm.count("model_features") ? vm["model_features"].as<std::vector<std::string>>() : std::vector<std::string>{"odom-bounds", "sc-sim", "alignment_quality"};

  pose_graph_par.est_output_dir = vm["output_directory"].as<std::string>() + "/est";
  pose_graph_par.gt_output_dir = vm["output_directory"].as<std::string>() + "/gt";
  pose_graph_par.eval_manager_par.loop_eval_output_dir = vm["output_directory"].as<std::string>() + "/loop";


  if(tbv_eval_par.debug_optimizer || slam_pars.loop_pars.debug_enabled){
    slam_pars.loop_pars.DMCP.MiniClosure_enabled = false;
    slam_pars.loop_pars.DGTVP.GTVicinity_enabled = false;
    slam_pars.loop_pars.DSCCP.SCClosure_enabled  = false;
  }

  if (vm.count("res"))
    par.res = vm["res"].as<double>();
  if (vm.count("min_distance"))
    rad_par.min_distance = vm["min_distance"].as<double>();
  if (vm.count("max_distance"))
    rad_par.max_distance = vm["max_distance"].as<double>();
  if (vm.count("job_nr"))
    eval_par.job_nr = vm["job_nr"].as<int>();
  if (vm.count("cost_type"))
    par.cost_type = vm["cost_type"].as<std::string>();
  if (vm.count("loss_type"))
    par.loss_type_ = vm["loss_type"].as<std::string>();
  if (vm.count("loss_limit"))
    par.loss_limit_ = vm["loss_limit"].as<double>();
  if (vm.count("covar_scale"))
    par.covar_scale_ = vm["covar_scale"].as<double>();
  if (vm.count("covar_sampling"))
    par.estimate_cov_by_sampling = vm["covar_sampling"].as<bool>();
  if (vm.count("covar_sample_save"))
    par.cov_samples_to_file_as_well = vm["covar_sample_save"].as<bool>();
  if (vm.count("covar_sample_dir"))
    par.cov_sampling_file_directory = vm["covar_sample_dir"].as<std::string>();
  if (vm.count("covar_XY_sample_range"))
    par.cov_sampling_xy_range = vm["covar_XY_sample_range"].as<double>();
  if (vm.count("covar_yaw_sample_range"))
    par.cov_sampling_yaw_range = vm["covar_yaw_sample_range"].as<double>();
  if (vm.count("covar_samples_per_axis"))
    par.cov_sampling_samples_per_axis = vm["covar_samples_per_axis"].as<int>();
  if (vm.count("covar_sampling_scale"))
    par.cov_sampling_covariance_scaler = vm["covar_sampling_scale"].as<double>();
  if (vm.count("regularization"))
    par.regularization_ = vm["regularization"].as<double>();
  if (vm.count("submap_scan_size"))
    par.submap_scan_size = vm["submap_scan_size"].as<int>();
  if (vm.count("k_strongest"))
    rad_par.k_strongest = vm["k_strongest"].as<int>();
  if (vm.count("registered_min_keyframe_dist"))
    par.min_keyframe_dist_= vm["registered_min_keyframe_dist"].as<double>();
  if (vm.count("est_directory"))
    eval_par.est_output_dir = vm["est_directory"].as<std::string>();
  if (vm.count("gt_directory"))
    eval_par.gt_output_dir = vm["gt_directory"].as<std::string>();
  if (vm.count("cfear_method_name"))
    eval_par.method = vm["cfear_method_name"].as<std::string>();
  if (vm.count("bag_path"))
    p.bag_file_path = vm["bag_path"].as<std::string>();
  if (vm.count("training_directory"))
    training_par.training_dir = vm["training_directory"].as<std::string>();
  if (vm.count("disable_training"))
    training_par.disable_training = vm["disable_training"].as<bool>();
  if (vm.count("radar_directory"))
    p.radar_dir = vm["radar_directory"].as<std::string>();
  if (vm.count("save_ROC"))
    training_par.save_ROC = vm["save_ROC"].as<bool>();
  if (vm.count("sequence"))
    eval_par.sequence = vm["sequence"].as<std::string>();
  if (vm.count("z-min"))
    rad_par.z_min = vm["z-min"].as<double>();
  if (vm.count("dataset"))
    rad_par.dataset = vm["dataset"].as<std::string>();
  if (vm.count("range-res"))
    rad_par.range_res = vm["range-res"].as<double>();
  if (vm.count("savepcd"))
    eval_par.save_pcd = true;
  if (vm.count("weight_option"))
    par.weight_opt = static_cast<weightoption>(vm["weight_option"].as<int>());
  if (vm.count("k_strongest"))
    rad_par.nb_guard_cells = vm["k_strongest"].as<int>();
  if (vm.count("regularization"))
    rad_par.false_alarm_rate = vm["regularization"].as<double>();
  if (vm.count("covar_scale"))
    rad_par.window_size = vm["covar_scale"].as<double>();

  if (vm.count("covar_sampling"))
    par.estimate_cov_by_sampling = vm["covar_sampling"].as<bool>();
  if (vm.count("covar_sample_save"))
    par.cov_samples_to_file_as_well = vm["covar_sample_save"].as<bool>();
  if (vm.count("covar_sample_dir"))
    par.cov_sampling_file_directory = vm["covar_sample_dir"].as<std::string>();
  if (vm.count("covar_XY_sample_range"))
    par.cov_sampling_xy_range = vm["covar_XY_sample_range"].as<double>();
  if (vm.count("covar_yaw_sample_range"))
    par.cov_sampling_yaw_range = vm["covar_yaw_sample_range"].as<double>();
  if (vm.count("covar_samples_per_axis"))
    par.cov_sampling_samples_per_axis = vm["covar_samples_per_axis"].as<int>();
  if (vm.count("covar_sampling_scale"))
    par.cov_sampling_covariance_scaler = vm["covar_sampling_scale"].as<double>();




  p.save_radar_img = vm["save_radar_img"].as<bool>();;
  rad_par.filter_type_ = Str2filter(vm["filter-type"].as<std::string>());
  par.store_graph = vm["store_graph"].as<bool>();;
  par.weight_intensity_ = vm["weight_intensity"].as<bool>();;
  par.compensate = !vm["disable_compensate"].as<bool>();
  par.use_guess = true; //vm["soft_constraint"].as<bool>();
  par.soft_constraint = false; // soft constraint is rarely useful, this is changed for testing of initi // vm["soft_constraint"].as<bool>();
  par.radar_ccw = vm["radar_ccw"].as<bool>();

  cout << "SLAM pars: \n" << slam_pars.ToString() << endl;
  cout << "Eval pars: \n" << tbv_eval_par.ToString() << endl;
  cout << "Pose graph pars: \n" << pose_graph_par.ToString() << endl;

  cout << "Output directory: " << tbv_eval_par.eval_output_dir << endl;

  std::ofstream ofs(tbv_eval_par.eval_output_dir+std::string("/pars.txt")); // Write
  ofs << tbv_eval_par.ToString() << pose_graph_par.ToString() << slam_pars.ToString() << endl;
  ofs.close();

}

class SLAMEval
{
public:
  SLAMEval(tbv_eval_parameters& eval_par, PoseGraph::Parameters& pose_graph_par, TBVSLAM::Parameters slam_pars, PoseGraphVis::Parameters& pose_vis_par):
    eval_par_(eval_par),
    pose_graph_par_(pose_graph_par),
    slam_pars_(slam_pars),
    last_config(tbv_slam::OptimizationParamsConfig()) {

    graph = PoseGraphPtr(new PoseGraph(pose_graph_par));
    vis = new PoseGraphVis(graph, pose_vis_par);
    slam = new TBVSLAM(graph, slam_pars_);
    
  }

  ~SLAMEval() {
    Halt(); // Wait for keypress if wait_for_key is set to true
    Save(); // Save evaluation)
    std::cout << "exited SLAMEval succesfully" << std::endl;
  }

  void Finish() {
    while(ros::ok() && !slam->FinishedLoopClosures()) {
      std::cout << "waiting to process all loop closures" << std::endl;
    }
  }

  void Save()
  {
    CFEAR_Radarodometry::timing.PresentStatistics();
    std::ofstream statistics_file;
    statistics_file.open (eval_par_.eval_output_dir + "/time_statistics.txt");
    statistics_file << CFEAR_Radarodometry::timing.GetStatistics();
    statistics_file.close();

    graph->Align();
    graph->SaveGraphString(eval_par_.eval_output_dir + "/graph.txt");
    graph->OutputGraph();
    if(eval_par_.save_graph){
      PoseGraph::SaveGraph(eval_par_.full_graph_path, graph);
    }

    graph->eval_manager.writeResultsToCSV( slam_pars_.loop_pars.namesToString(), slam_pars_.loop_pars.valuesToString() );
    cout << "Analyze loop closures: " << endl;
    cout << "roscd place_recognition_radar/python/ && python3 LoopClosureEval.py  --output_folder " <<  pose_graph_par_.eval_manager_par.loop_eval_output_dir << " --csv_file " << pose_graph_par_.eval_manager_par.loop_eval_output_dir << "/loop.csv --p-threshold 0.8" << endl;
  }

  void callback(tbv_slam::OptimizationParamsConfig &config, uint32_t level)
  {
    static bool first = true;
    if(first)
    {
      first = false;
      return;
    }
    last_config = config;
    parameters_updated = true;
  }

  void updateGraphConstraintInformation(){
    cout << "Locking graph" << endl;
    graph->m_graph.lock();
    graph->par_.opt_pars.ceres_pars.scaling_pars = last_config;
    cout << "Unlocking graph" << endl;
    graph->m_graph.unlock();
  }

  void Halt()
  {
    ros::Rate r(0.2);
    while(ros::ok() && eval_par_.wait_for_key)
    {
      vis->ProcessFrame();
      r.sleep();
      cout << "press e to (e)xit" << endl;
      char c = getchar();
      if(c =='e' || c=='E')
        break;
    }
  }

  tbv_eval_parameters eval_par_;
  PoseGraph::Parameters pose_graph_par_;
  TBVSLAM::Parameters slam_pars_;
  PoseGraphPtr graph;
  PoseGraphVis* vis;
  TBVSLAM* slam ;
  bool parameters_updated = false;
  tbv_slam::OptimizationParamsConfig last_config;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tbv_slam");


  OdometryKeyframeFuser::Parameters odom_pars;
  radarDriver::Parameters rad_pars;
  EvalTrajectory::Parameters eval_pars;
  cfear_eval_parameters eval_p;
  training_parameters training_pars;

  tbv_eval_parameters tbv_eval_par;
  PoseGraph::Parameters pose_graph_par;
  TBVSLAM::Parameters slam_pars;
  PoseGraphVis::Parameters pose_vis_par;

  //ReadOptions(argc, argv);
  ReadOptions(argc, argv, odom_pars, rad_pars, eval_pars, eval_p, training_pars, tbv_eval_par, pose_graph_par, slam_pars, pose_vis_par);

  std::ofstream ofs_before(eval_pars.est_output_dir+std::string("../pars.txt")); // Write
  std::string par_str_before = rad_pars.ToString()+odom_pars.ToString()+eval_pars.ToString()+"nr_frames, "+std::to_string(0)+"\n"+CFEAR_Radarodometry::timing.GetStatistics();
  cout<<"Odometry parameters:\n" << par_str_before<<endl;
  ofs_before<<par_str_before<<endl;
  ofs_before.close();

  SLAMEval eval(tbv_eval_par, pose_graph_par, slam_pars, pose_vis_par);

  radarReader reader(odom_pars, rad_pars, eval_pars, eval_p, training_pars, eval.graph);
  
  eval.Finish();

  std::cout << "output dir: " << eval_pars.est_output_dir << std::endl;
  std::cout << "tbv output dir: " << tbv_eval_par.eval_output_dir << std::endl;

  std::ofstream ofs(eval_pars.est_output_dir+std::string("../pars.txt")); // Write
  std::string par_str = rad_pars.ToString()+odom_pars.ToString()+eval_pars.ToString()+"\nnr_frames, "+std::to_string(reader.GetSize())+"\n"+CFEAR_Radarodometry::timing.GetStatistics();
  ofs<<par_str<<endl;
  ofs.close();

  return 0;
}



