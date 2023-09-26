#include "tbv_slam/ceresoptimizer.h"

namespace tbv_slam {

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.

CeresLeastSquares::CeresLeastSquares(ConstraintsHandler& constraints,
                                     RadarScanHandler& nodes,
                                     const CeresLeastSquares::Parameters& pars) : constraints_(constraints), nodes_(nodes), pars_(pars) {
}

void CeresLeastSquares::Solve(){
  BuildOptimizationProblem();
  SolveOptimizationProblem();
}

CeresLeastSquares::Parameters::Parameters(){
  scaling_pars.loop_vxx = 0.01;
  scaling_pars.loop_vyy = 0.01;
  scaling_pars.loop_vtt = 0.001;
  scaling_pars.odom_vxx = 0.01;
  scaling_pars.odom_vyy = 0.01;
  scaling_pars.odom_vtt = 0.001;
  scaling_pars.loop_scaling = 500000;
  scaling_pars.replace_cov_by_identity = true;
}
void CeresLeastSquares::BuildOptimizationProblem(){
  CHECK(nodes_.size()!= 0);
  CHECK (constraints_.size() != 0);

  //cout << "scaling_pars.replace_cov_by_identity " << pars_.scaling_pars.replace_cov_by_identity << endl;
  //cout << "scaling_pars.loop_scaling " << std::boolalpha << pars_.scaling_pars.loop_scaling  << endl;
  AddConstraintType(odometry, nullptr);
  AddConstraintType(loop_appearance, new ceres::CauchyLoss(0.1));

  auto pose_last_iter = nodes_.begin();
  problem_.SetParameterBlockConstant(pose_last_iter->second.T.p.data());
  problem_.SetParameterBlockConstant(pose_last_iter->second.T.q.coeffs().data());

  ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;
  for(auto itr = nodes_.begin() ; itr != nodes_.end() ; itr++){
    problem_.SetParameterization(itr->second.T.q.coeffs().data(),
                                 quaternion_local_parameterization);
    problem_.SetParameterization(itr->second.T.q.coeffs().data(),
                                 quaternion_local_parameterization);
  }
}

bool CeresLeastSquares::SolveOptimizationProblem() {

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);
  //std::cout << summary.FullReport() << '\n';
  std::cout << summary.BriefReport() << '\n';
  //std::cout << summary.FullReport() << '\n';
  //std::cout<<"Optimized"<<std::endl;
  return summary.IsSolutionUsable();
}

void CeresLeastSquares::AddConstraintType(const ConstraintType& ct,
                                          ceres::LossFunction* loss_function
                                          ){

  if(pars_.scaling_pars.replace_cov_by_identity){
    //cout << "Optimize with FIXED covariances: " << endl;
  }else{
    //cout << "Optimize with DYNAMIC covariances: " << std::boolalpha <<pars_.scaling_pars.replace_cov_by_identity << endl;
  }

  for (auto c_itr = constraints_.begin(ct); c_itr != constraints_.end(ct); c_itr++) {
    if( !nodes_.NodeExists(c_itr->second.id_begin) || !nodes_.NodeExists(c_itr->second.id_begin) )
      throw std::invalid_argument("BuildOptimizationProblem - Nodes doesn't exist.");

    auto pose_begin_iter = nodes_.Get(c_itr->second.id_begin);
    auto pose_end_iter = nodes_.Get(c_itr->second.id_end);

    Constraint3d& constraint = c_itr->second;

    Eigen::Matrix<double,6,1> scale_diag;
    scale_diag << 1.0/pars_.scaling_pars.odom_vxx, 1.0/pars_.scaling_pars.odom_vyy, 1, 1, 1, 1.0/pars_.scaling_pars.odom_vtt;

    Eigen::Matrix<double, 6, 6> I_scaled;
    const double loop_scale_factor = (ct ==loop_appearance) ?  1.0/pars_.scaling_pars.loop_scaling : 1.0 ;
    if(pars_.scaling_pars.replace_cov_by_identity){
       I_scaled = Eigen::Matrix<double, 6, 6>::Identity()*scale_diag.asDiagonal()*loop_scale_factor;
    }
    else {
       I_scaled = constraint.information*loop_scale_factor;
    }
    if(std::next(constraints_.begin(ct)) == c_itr){
      //cout << Constraint2String(ct) << "orig: \n" << constraint.information.inverse() << endl;
      //cout << Constraint2String(ct) << "scaled: \n" << I_scaled.inverse() << endl;
    }


    const Eigen::Matrix<double, 6, 6> sqrt_information = I_scaled.llt().matrixL();
    ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);
    problem_.AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.T.p.data(),
                              pose_begin_iter->second.T.q.coeffs().data(),
                              pose_end_iter->second.T.p.data(),
                              pose_end_iter->second.T.q.coeffs().data());
  }
}


// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, RadarScanHandler& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (auto itr = poses.begin(); itr!=poses.end(); itr++) {

    outfile << itr->second.idx_ << " " << itr->second.T.p.transpose() << " "
            << itr->second.T.q.x() << " " << itr->second.T.q.y() << " "
            << itr->second.T.q.z() << " " << itr->second.T.q.w() << '\n';
  }
  return true;
}


}
