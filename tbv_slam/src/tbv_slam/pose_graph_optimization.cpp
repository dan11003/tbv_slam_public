
#include "tbv_slam/pose_graph_optimization.h"


namespace tbv_slam {

PoseGraphHandler::PoseGraphHandler(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars){
  solver_ = CreatePGSolver(constraints, nodes, pars);
}

AdapterCeresLeastSquares::AdapterCeresLeastSquares(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars) : PGAdapter(constraints, nodes, pars), solver_(constraints, nodes, pars.ceres_pars){
}

void AdapterCeresLeastSquares::Solve(){
  solver_.Solve();
}

std::shared_ptr<PGAdapter> PoseGraphHandler::CreatePGSolver(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars){
  if(pars.solver == PoseGraphSolver::CeresLeastSquares_E){
    auto adapter = new AdapterCeresLeastSquares(constraints, nodes, pars);
    return std::shared_ptr<AdapterCeresLeastSquares> (adapter);
    //return std::make_shared<AdapterCeresLeastSquares>( AdapterCeresLeastSquares(constraints, nodes, pars));
  }
  else{
    //auto adapter = new AdapterCeresLeastSquares(constraints, nodes, pars);
    //std::shared_ptr<AdapterCeresLeastSquares>(adapter);
    //return std::make_shared<AdapterCeresLeastSquares>( AdapterCeresLeastSquares(constraints, nodes, pars));
  }
}


}
