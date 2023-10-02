
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

class eval_parameters
{
public:
  eval_parameters() {}
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


void ReadOptions(const int argc, char**argv, eval_parameters& eval_par, PoseGraph::Parameters& pose_graph_par, TBVSLAM::Parameters& slam_pars, PoseGraphVis::Parameters& pose_vis_par){

  po::options_description desc{"Options"};
  desc.add_options()
      ("help,h", "Help screen")
      ("visualize", po::value<bool>()->default_value(false),"visualize")
      ("miniloop-enabled", po::value<bool>()->default_value(false),"short distance miniloop closure")
      ("wait_for_key", po::value<bool>()->default_value(false),"wait for keypress")
      ("input_directory", po::value<std::string>()->default_value(""), "directory of odometry sequences")
      ("dataset", po::value<std::string>()->default_value(""), "dataset")
      ("sequence", po::value<std::string>()->default_value(""), "sequence")
      ("output_directory", po::value<std::string>()->default_value(""), "eval_output_dir")
      ("experiment_name", po::value<std::string>()->default_value(""), "eval_output_dir")
      ("method_name", po::value<std::string>()->default_value("TBV SLAM-8"), "method name")
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


  eval_par.input_directory = vm["input_directory"].as<std::string>();
  eval_par.dataset = vm["dataset"].as<std::string>();
  pose_graph_par.eval_manager_par.dataset = vm["dataset"].as<std::string>();
  pose_graph_par.eval_manager_par.sequence = vm["sequence"].as<std::string>();
  eval_par.sequence = vm["sequence"].as<std::string>();
  eval_par.eval_output_dir = vm["output_directory"].as<std::string>();
  eval_par.simple_graph_path = eval_par.input_directory + "/" + eval_par.dataset + "/" + eval_par.sequence + "/est/" +  vm["simple_graph_name"].as<std::string>();
  eval_par.full_graph_path = eval_par.input_directory + "/" + eval_par.dataset + "/" + eval_par.sequence + "/est/" +  vm["full_graph_name"].as<std::string>();
  eval_par.save_graph = vm["save_graph"].as<bool>();
  eval_par.load_graph = vm["load_graph"].as<bool>();

  slam_pars.loop_pars.gt_loop = vm["gt_loop"].as<bool>();
  pose_graph_par.disable_optimization = vm["optimization_disabled"].as<bool>();
  eval_par.debug_optimizer = slam_pars.loop_pars.debug_enabled = vm["debug_enabled"].as<bool>();
  slam_pars.loop_pars.threading = false;
  pose_graph_par.online = false;

  slam_pars.loop_pars.data_dir = eval_par.input_directory + "/" + eval_par.dataset + "/" + eval_par.sequence + "/radar" ;

  if( vm["use_default_alignment_data"].as<bool>() ){
    slam_pars.loop_pars.training_data_dir = ros::package::getPath("tbv_slam") + "/model_parameters/";
  }
  else{
    slam_pars.loop_pars.training_data_dir = eval_par.input_directory + "/" + eval_par.dataset + "/" + eval_par.sequence + "/training" ;
  }


  slam_pars.loop_pars.visualize = eval_par.visualize = pose_vis_par.visualization_enabled_ = vm["visualize"].as<bool>();
  eval_par.wait_for_key = vm["wait_for_key"].as<bool>();
  eval_par.experiment_name = vm["experiment_name"].as<std::string>();
  eval_par.method = vm["method_name"].as<std::string>();
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


  if(eval_par.debug_optimizer || slam_pars.loop_pars.debug_enabled){
    slam_pars.loop_pars.DMCP.MiniClosure_enabled = false;
    slam_pars.loop_pars.DGTVP.GTVicinity_enabled = false;
    slam_pars.loop_pars.DSCCP.SCClosure_enabled  = false;
  }

  cout << "SLAM pars: \n" << slam_pars.ToString() << endl;
  cout << "Eval pars: \n" << eval_par.ToString() << endl;
  cout << "Pose graph pars: \n" << pose_graph_par.ToString() << endl;

  cout << "Output directory: " << eval_par.eval_output_dir << endl;

  std::ofstream ofs(eval_par.eval_output_dir+std::string("/pars.txt")); // Write
  ofs << eval_par.ToString() << pose_graph_par.ToString() << slam_pars.ToString() << endl;
  ofs.close();

}

class SLAMEval
{
public:
  SLAMEval(eval_parameters& eval_par, PoseGraph::Parameters& pose_graph_par, TBVSLAM::Parameters slam_pars, PoseGraphVis::Parameters& pose_vis_par):
    eval_par_(eval_par),
    pose_graph_par_(pose_graph_par),
    slam_pars_(slam_pars),
    last_config(tbv_slam::OptimizationParamsConfig()) {
    Load();
    vis = new PoseGraphVis(graph, pose_vis_par);
    slam = new TBVSLAM(graph, slam_pars_);

    if(eval_par_.debug_optimizer){
      RunDebugPoseGraphOptimization();
    }
    else{
      RunBasicEvaluation();
    }
    Halt(); // Wait for keypress if wait_for_key is set to true
    Save(); // Save evaluation
  }

  void Load()
  {
    if(!eval_par_.load_graph) {
      cout << "Loading the simple graph: " << eval_par_.simple_graph_path << endl;
      PoseGraph::LoadSimpleGraph(eval_par_.simple_graph_path, graph, pose_graph_par_);
    }else{
      cout << "Loading the full graph: " << eval_par_.full_graph_path << endl;
      PoseGraph::LoadGraph(eval_par_.full_graph_path, graph);
      graph->par_ = pose_graph_par_;
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

  /*******************  Experiment 1 ********************/
  void RunBasicEvaluation()
  {
    cout << "RunBasicEvaluation" << endl;
    while ( ros::ok() && slam->ProcessFrame(/*optimize*/false,  /*loopclosure*/true) )
    {
      ros::spinOnce();
    }
    // If wait_for_key, wait for input before optimize the graph
    if(eval_par_.wait_for_key){
      cout << "Press Enter to optimize graph:" << endl;
      getchar();
    }
    if(!pose_graph_par_.disable_optimization){
      slam->ProcessFrame(/*debug_optimizer*/true , /*loopclosure*/false);
      graph->Align();
    }
  }/*******************  Experiment 1 ********************/


  /*******************  Experiment 2 ********************/
  void RunDebugPoseGraphOptimization()
  {
    cout << "RunDebugPoseGraphOptimization" << endl;
    dynamic_reconfigure::Server<tbv_slam::OptimizationParamsConfig> server;
    dynamic_reconfigure::Server<tbv_slam::OptimizationParamsConfig>::CallbackType callback_function;
    callback_function = boost::bind(&SLAMEval::callback, this, _1, _2);
    server.setCallback(callback_function);
    while (ros::ok())
    {
      ros::spinOnce(); // The graph optimization is forced here or in the callback
      ros::Duration(0.01).sleep();
      if(parameters_updated){
        cout << "Update Graph" << endl;
        updateGraphConstraintInformation();
        cout << "Optimize" << endl;
        slam->ProcessFrame(/*optimize*/true, /*loopclosure*/false);
        graph->Align();
        parameters_updated = false;
      }
    }
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
  /*******************  end Experiment 2 ********************/


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

  eval_parameters  eval_par_;
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

  eval_parameters eval_par;
  PoseGraph::Parameters pose_graph_par;
  TBVSLAM::Parameters slam_pars;
  PoseGraphVis::Parameters pose_vis_par;
  ReadOptions(argc, argv, eval_par, pose_graph_par, slam_pars, pose_vis_par);
  SLAMEval eval(eval_par, pose_graph_par, slam_pars, pose_vis_par);

  return 0;
}



