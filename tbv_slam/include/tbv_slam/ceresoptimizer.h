#pragma once

#include "ceres/ceres.h"
#include "ceres/autodiff_cost_function.h"
#include "cfear_radarodometry/types.h"
#include "Eigen/Core"
#include <tbv_slam/OptimizationParamsConfig.h>

namespace tbv_slam {
using namespace Eigen;
using namespace CFEAR_Radarodometry;

class CeresLeastSquares
{
public:

  class Parameters{
  public:
    Parameters();

    tbv_slam::OptimizationParamsConfig scaling_pars;
  };

public:
  CeresLeastSquares(ConstraintsHandler& constraints,
                    RadarScanHandler& nodes,
                    const CeresLeastSquares::Parameters& pars);

  void Solve();

private:

  void AddConstraintType(const ConstraintType& ct,
                         ceres::LossFunction* loss_function);

  void BuildOptimizationProblem();

  bool SolveOptimizationProblem();

  ceres::Problem problem_;
  ConstraintsHandler& constraints_;
  RadarScanHandler& nodes_;
  Parameters pars_;



};



class PoseGraph3dErrorTerm {
public:
  PoseGraph3dErrorTerm(const Pose3d& t_ab_measured,
                       const Eigen::Matrix<double, 6, 6>& sqrt_information)
    : t_ab_measured_(t_ab_measured), sqrt_information_(sqrt_information) {}
  template <typename T>
  bool operator()(const T* const p_a_ptr, const T* const q_a_ptr,
                  const T* const p_b_ptr, const T* const q_b_ptr,
                  T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_b(p_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T> > q_b(q_b_ptr);
    // Compute the relative transformation between the two frames.
    Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
    Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;
    // Represent the displacement between the two frames in the A frame.
    Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);
    // Compute the error between the two orientation estimates.
    Eigen::Quaternion<T> delta_q =
        t_ab_measured_.q.template cast<T>() * q_ab_estimated.conjugate();
    // Compute the residuals.
    // [ position         ]   [ delta_p          ]
    // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
    Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) =
        p_ab_estimated - t_ab_measured_.p.template cast<T>();
    residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();
    // Scale the residuals by the measurement uncertainty.
    residuals.applyOnTheLeft(sqrt_information_.template cast<T>());
    return true;
  }
  static ceres::CostFunction* Create(
      const Pose3d& t_ab_measured,
      const Eigen::Matrix<double, 6, 6>& sqrt_information) {
    return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
          new PoseGraph3dErrorTerm(t_ab_measured, sqrt_information));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    // The measurement for the position of B relative to A in the A frame.
    const Pose3d t_ab_measured_;
  // The square root of the measurement information matrix.
  const Eigen::Matrix<double, 6, 6> sqrt_information_;
};


}

