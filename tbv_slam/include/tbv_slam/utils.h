#pragma once
#include "cfear_radarodometry/types.h"
#include "string"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
//#include "alignment_checker/alignmentinterface.h"


namespace tbv_slam {

using namespace CFEAR_Radarodometry;

class RadarReader
{
public:

  RadarReader() {}

  virtual bool Get(unsigned long stamp, cv::Mat& radar_scan_img) = 0;

};

class PNGReaderInterface : public RadarReader
{
public:

  PNGReaderInterface(const std::string& folder) : folder_(folder) {}

  bool Get(unsigned long stamp, cv::Mat& radar_scan_img);

private:
  std::string folder_;
};

typedef struct loopStatistics{
  double loop_distance;
  double closest_distance;
  unsigned int guess_nr;
  unsigned int idfrom, idto;

}LoopStatistics;

#define ODOM_BOUNDS "odom-bounds"
#define SC_SIM "sc-sim"
#define CFEAR_COST "CFEAR"
#define CORAL_COST "coral"
#define COMBINED_COST "alignment_quality"


Constraint3d CreateAppearanceConstraint(const unsigned long ibegin, const unsigned long iend, const double apperance_similarity, const Eigen::Affine3d& Tguess = Eigen::Affine3d::Identity());

Constraint3d CreateMiniloopConstraint(const unsigned long ibegin, const unsigned long iend);

Constraint3d CreateCandidateConstraint(const unsigned long ibegin, const unsigned long iend, const std::string& description = "");


}
