#pragma once
#include "Eigen/Core"

#include "iostream"
#include "stdio.h"
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>

//#include "common/read_g2o.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "cfear_radarodometry/types.h"
#include "tbv_slam/ceresoptimizer.h"
#include <tbv_slam/OptimizationParamsConfig.h>
#include <memory.h>

namespace tbv_slam {
using namespace Eigen;
using namespace CFEAR_Radarodometry;
using tbv_slam::CeresLeastSquares;



typedef enum {CeresLeastSquares_E = 0 }PoseGraphSolver;

struct PGOptimizationParameters{


  PoseGraphSolver solver;

  CeresLeastSquares::Parameters ceres_pars;

  void GetParametersFromRos( ros::NodeHandle& param_nh){}

  std::string ToString(){return std::string();}

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {

  }
};



/* PURE VIRTUAL */
class PGAdapter
{
public:
  PGAdapter(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars) : constraints_(constraints), nodes_(nodes), pars_(pars){}

  virtual void Solve() {};


  PGOptimizationParameters pars_;
  ConstraintsHandler& constraints_;
  RadarScanHandler& nodes_;

};

class AdapterCeresLeastSquares : public PGAdapter
{
public:
  AdapterCeresLeastSquares(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars);

  void Solve();

  class CeresLeastSquares solver_;
};






class PoseGraphHandler{
  public:
  PoseGraphHandler(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars);

  std::shared_ptr<PGAdapter> CreatePGSolver(ConstraintsHandler& constraints, RadarScanHandler& nodes, const PGOptimizationParameters& pars);

  std::shared_ptr<PGAdapter> solver_;
};


}
