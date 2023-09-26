#include "tbv_slam/utils.h"
namespace tbv_slam {

bool PNGReaderInterface::Get(unsigned long stamp, cv::Mat& radar_scan_img){

  const std::string img_path = folder_ + "/" + std::to_string(stamp) + ".png";
  try{
    radar_scan_img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
  }
  catch (const std::exception& e) {
    std::cout << "Cannot load: " << img_path << std::endl;
    throw std::string("PNGReaderInterface::Get(unsigned long stamp, cv::Mat& radar_scan_img) Load Error");
  }
  if (radar_scan_img.rows < radar_scan_img.cols)
    radar_scan_img = radar_scan_img.t();

  return radar_scan_img.rows*radar_scan_img.cols > 0;

}



Constraint3d CreateAppearanceConstraint(const unsigned long ibegin, const unsigned long iend,const double apperance_similarity, const Eigen::Affine3d& Tguess){
  Constraint3d c;
  c.id_begin = ibegin; c.id_end = iend; c.type = loop_appearance;  c.quality[SC_SIM] = apperance_similarity;
  c.t_be = PoseEigToCeres(Tguess);
  c.quality = {{ODOM_BOUNDS,0.0},{SC_SIM,apperance_similarity},{COMBINED_COST,0.0}};
  return c;
}
Constraint3d CreateMiniloopConstraint(const unsigned long ibegin, const unsigned long iend){
  Constraint3d c;
  c.id_begin = ibegin; c.id_end = iend;
  c.quality = {{ODOM_BOUNDS,0.0},{SC_SIM,0.0},{COMBINED_COST,0.0}};
  return c;
}
Constraint3d CreateCandidateConstraint(const unsigned long ibegin, const unsigned long iend, const std::string& description){
  Constraint3d c;
  c.id_begin = ibegin; c.id_end = iend; c.t_be = Pose3d::Identity(); c.information = Eigen::Matrix<double,6,6>::Identity(); c.type = candidate; c.info = description;
  c.quality = {{ODOM_BOUNDS,0.0},{SC_SIM,0.0},{COMBINED_COST,0.0}};
  return c;
}



}
