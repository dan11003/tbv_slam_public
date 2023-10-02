#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "place_recognition_radar/nanoflann.hpp"
#include "place_recognition_radar/KDTreeVectorOfVectorsAdaptor.h"

#include "place_recognition_radar/tictoc.h"
#include "unordered_map"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


// namespace SC2
// {

void coreImportTest ( void );

Eigen::Affine3d VectorToAffine3dxyez(const std::vector<double>& vals);

// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );

void VisualizeScanContext(MatrixXd& M, const std::string& window_name, const bool visualize = false);
void SaveScanContext(MatrixXd& M, const std::string& name, const bool normalize = false);

typedef struct  cand{
double min_dist;
double min_dist_sc;
double min_dist_odom;
float yaw_diff_rad;
int nn_idx;
int argmin_shift;
Eigen::Affine3d Taug;
}candidate;

bool CandidateDistanceCloser(candidate const& lhs, candidate const& rhs);


class SCManager
{
public: 
    SCManager( ) = default; // reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.
    // ScManager(){}
    // ScManager( const int pc_num_ring = 40 ):PC_NUM_RING(pc_num_ring){}

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down );
    std::vector<candidate> detectLoopClosureID( void ); // int: nearest node index, float: relative yaw

public:
    // hyper parameters ()
    const double LIDAR_HEIGHT = 2.0; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    int    N_candidates = 1; // 20 in the original paper (IROS 18)
    int    PC_NUM_RING = 40; // 20 in the original paper (IROS 18)
    int    PC_NUM_SECTOR = 120; // 60 in the original paper (IROS 18)
    double PC_MAX_RADIUS = 80.0; // 80 meter max in the original paper (IROS 18)
    double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
    double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);

    // tree
    int    NUM_EXCLUDE_RECENT = 0; // simply just keyframe gap, but node position distance-based exclusion is ok. ### MODIFIED BY ADOLFSSON - 50 down to 15
    int    NUM_CANDIDATES_FROM_TREE = 10; // 10 is enough. (refer the IROS 18 paper)
    double DISTANCE_EXCLUDE_RECENT = 10.0;

    // loop thres
    double SEARCH_RATIO = 0.1; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    // const double SC_DIST_THRES = 0.13; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    double SC_DIST_THRES = 0.2; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    const int    TREE_MAKING_PERIOD_ = 50; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
    int          tree_making_period_conter = 0;

    // data 
    std::vector<Eigen::MatrixXd> polarcontexts_;        // SC Descriptor
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_; // Ringkey descriptor


    KeyMat polarcontext_invkeys_mat_; // Ringkey descriptor (std::vector)
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

}; // SCManager

// } // namespace SC2
