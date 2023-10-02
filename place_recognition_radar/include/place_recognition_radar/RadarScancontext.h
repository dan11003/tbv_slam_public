#pragma once

#include "place_recognition_radar/Scancontext.h"
#include <algorithm>
#include <ros/ros.h>

// SRVS
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "cfear_radarodometry/utils.h"
#include <pcl_ros/transforms.h>
namespace PlaceRecognitionRadar
{

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};





using CFEAR_Radarodometry::Join;


class RSCManager : public SCManager
{
public:
    struct Parameters
    {
        double radar_threshold = 0;
        int PC_NUM_RING = 40;
        int PC_NUM_SECTORS = 120;
        double PC_MAX_RADIUS = 80;
        double SEARCH_RATIO = 0.1;
        int NUM_CANDIDATES_FROM_TREE = 10;
        double SC_DIST_THRES = 0.2;
        int N_CANDIDATES = 1;
        bool prints = true;
        bool normalize = false;
        std::string interpolation = "area";
        bool desc_plot = false;
        double odom_sigma_error = 0.05;
        bool odometry_coupled_closure = true;
        bool augment_sc = true;
        double no_point = 0;
        std::string desc_function = "sum";
        double desc_divider = 1;
        const std::string namesToString() const
        {
            return Join(GetStrMap(), true);
        }

        const std::string valuesToString() const
        {
            return Join(GetStrMap(), false);
        }

        const std::map<std::string,std::string> GetStrMap()const{
          return std::map<std::string,std::string>({
            {"SC - radar_threshold",std::to_string(radar_threshold)},
            {"SC - PC_NUM_RING",std::to_string(PC_NUM_RING)},
            {"SC - PC_NUM_SECTORS",std::to_string(PC_NUM_SECTORS)},
            {"SC - SEARCH_RATIO",std::to_string(SEARCH_RATIO)},
            {"SC - NUM_CANDIDATES_FROM_TREE",std::to_string(NUM_CANDIDATES_FROM_TREE)},
            {"SC - SC_DIST_THRES",std::to_string(SC_DIST_THRES)},
            {"SC - N_CANDIDATES",std::to_string(N_CANDIDATES)},
            {"SC - prints",std::to_string(prints)},
            {"SC - normalize",std::to_string(normalize)},
            {"SC - interpolation",interpolation},
            {"SC - desc_plot",std::to_string(desc_plot)},
            {"SC - odom_sigma_error",std::to_string(odom_sigma_error)},
            {"SC - odometry_coupled_closure",std::to_string(odometry_coupled_closure)},
            {"SC - augment_sc",std::to_string(augment_sc)},
            {"SC - no_point",std::to_string(no_point)},
            {"SC - desc_function",desc_function},
            {"SC - desc_divider",std::to_string(desc_divider)}
          });
        }
    };

    RSCManager(const RSCManager::Parameters& pars);
    ~RSCManager(){}

    // Chagned to handle cv mat files
    Eigen::MatrixXd MakeRadarContext(cv::Mat& scan_img);

    Eigen::MatrixXd MakeRadarCloudContext(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    void makeAndSaveScancontextAndKeysRadarRaw(cv::Mat& scan_mat, const Eigen::Affine3d& Todom);

    void makeAndSaveScancontextAndKeysRadarCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Affine3d& Todom, bool augment = true);

    // Change to write min dist to input parameter
    std::vector<candidate> detectLoopClosureID(double& min_dist_ref); // int: nearest node index, float: relative yaw

    std::vector<candidate> detectLoopClosureID(){double discard ; return detectLoopClosureID(discard);}

    Parameters par;
private:

    void makeAndSaveScancontextAndKeys(Eigen::MatrixXd& sc, const Eigen::Affine3d& Todom);

    void ExcludeAndUpdateLikelihood(const Eigen::Affine3d& Todom);

    void VanillaKDNNSearch(std::vector<size_t>& candidate_indexes, const std::vector<float>& curr_key);

    void OdometryNNSearch(std::vector<size_t>& candidate_indexes, const std::vector<float>& curr_key);

    inline double L2norm(const std::vector<float>& v1, const std::vector<float>& v2);

    int interpolation_type;
    std::vector<Eigen::Affine3d> odom_poses_;

    std::vector<double> odom_similarity;

    typedef struct s_augments{
      Eigen::MatrixXd desc;
      Eigen::MatrixXd ring_key;
      std::vector<float> ring_key_std;
      Eigen::Affine3d Toffset;
    }augments;

    std::vector<augments> current_and_augments_;

}; // RSCManager



}
