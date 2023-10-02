#include "tbv_slam/tbv_slam.h"
namespace tbv_slam {

void TBVSLAM::Parameters::GetParametersFromRos( ros::NodeHandle& param_nh){
  // param_nh.param<std::string>("input_points_topic", input_points_topic, "/Navtech/Filtered");
}


TBVSLAM::TBVSLAM(PoseGraphPtr graph, const TBVSLAM::Parameters& par) : par_(par), graph_(graph)
{
  //loop_closures_.push_back(boost::unique_ptr<loopclosure>(new ));
  //loop_closures_.push_back( boost::make_shared<ScanContextClosure>(ScanContextClosure(graph_,par_.loop_pars)) );
  //loop_closures_.push_back( boost::shared_ptr<SimpleDistanceClosure>(new SimpleDistanceClosure(graph_, par_.loop_pars)) );
  if (par_.loop_pars.DMCP.MiniClosure_enabled){
    cout << "starting : MiniClosure " << endl;
    loop_closures_.push_back( boost::shared_ptr<MiniClosure>(new MiniClosure(graph_, par_.loop_pars)) );
  }
  if(par_.loop_pars.DGTVP.GTVicinity_enabled){
    cout << "starting : GTVicinityClosure " << endl;
    loop_closures_.push_back( boost::shared_ptr<GTVicinityClosure>(new GTVicinityClosure(graph_, par_.loop_pars)) );
  }
  if(par_.loop_pars.DSCCP.SCClosure_enabled){
    cout << "starting : ScanContextClosure " << endl;
    loop_closures_.push_back( boost::shared_ptr<ScanContextClosure>(new ScanContextClosure(graph_, par_.loop_pars)) );
  }
  if(par_.loop_pars.debug_enabled){
    cout << "starting : debug_enabled " << endl;
    loop_closures_.push_back( boost::shared_ptr<DebugConstraints>(new DebugConstraints(graph_, par_.loop_pars)) );
  }
}

bool TBVSLAM::ProcessFrame(const bool optimize, const bool loopclosure){
  bool still_running = false;
  if(loopclosure){
    for (auto && l : loop_closures_){
      still_running = still_running || l->ProcessFrame(); // true if anything (||) is running
    }
  }
  if(optimize){
    graph_->ForceOptimize();
  }
  return still_running;
}

bool TBVSLAM::FinishedLoopClosures() {
  ros::Duration(1).sleep();
  return loop_closures_.front()->FinishedLoopClosures();
}

}
