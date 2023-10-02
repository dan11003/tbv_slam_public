#pragma once


#include "memory.h"
//Eigen
#include "Eigen/Dense"

//PCL
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"

// OpenCv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "pcl/common/transforms.h"
#include "pcl/kdtree/flann.h"
#include "pcl/kdtree/kdtree.h"
#include "memory.h"
#include <iomanip>
#include "cfear_radarodometry/n_scan_normal.h"

// ROS tf
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "cfear_radarodometry/statistics.h"
//Coral includes
#include "alignment_checker/Utils.h"
#include "alignment_checker/ScanType.h"
// #include "alignment_checker/ScanEvaluator.h"

// MSGS
#include "std_msgs/Float64MultiArray.h"

// SRVS
#include "alignment_checker/AlignmentData.h"
#include "alignment_checker/AlignmentDataRequest.h"

namespace CorAlignment{

using std::endl;
using std::cout;
using std::cerr;

///////////// BASE /////////////////


class AlignmentQuality
{
public:
    class parameters
    {
    public:

        typedef enum entropy_config{ any=0, non_zero=1, abs=2, kl=3 }entropy_cfg;

        parameters() {}

        std::string method = "P2L";
        double radius = 3;
        entropy_cfg ent_cfg = any;
        bool weight_res_intensity = false;
        bool output_overlap = true;
        bool visualize = false;

        static const std::vector<std::string> HeaderToString(){
            return { "method" , "radius" , "entropy_setting" };
        }

        const std::vector<std::string> ValsToString() const{
            return { method , std::to_string(radius) , std::to_string(ent_cfg) };
        }

        const std::string ToString(){
            std::ostringstream stringStream;
            std::vector<std::string> header = HeaderToString();
            std::vector<std::string> values = ValsToString();
            if(header.size() != values.size())
                throw std::runtime_error("size error ToString");
            for(int i = 0 ; i<header.size() ; i++)
                stringStream << header[i] << "\t" << values[i] <<endl;
            return stringStream.str();
        }
    };

    AlignmentQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : par_(par), Toffset_(Toffset) {
        src_ = src;
        ref_ = ref;
        quality_ = {0,0,0};
        residuals_ = {0,0,0};
    }

    void Visualize();

    virtual ~AlignmentQuality(){}

    virtual std::vector<double> GetResiduals() {return residuals_;}

    virtual std::vector<double> GetQualityMeasure(){return quality_;}

    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);

    std::shared_ptr<PoseScan> src_, ref_;
    AlignmentQuality::parameters par_;
    const Eigen::Affine3d Toffset_;
    std::vector<double> quality_;
    std::vector<double> residuals_;
    bool valid_ = false;



};
typedef std::shared_ptr<AlignmentQuality> AlignmentQuality_S;

/************** P2P ********************/

class p2pQuality: public AlignmentQuality
{
public:

    p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

    ~p2pQuality(){}

    std::vector<double> GetResiduals() {
        return residuals_;
        //return {0,0,0};
    }

    std::vector<double> GetQualityMeasure();

protected:
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

class keypointRepetability: public AlignmentQuality
{
public:

    keypointRepetability(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

    ~keypointRepetability(){}

protected:
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_src_, kdtree_ref_;
    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

/************** P2D ********************/

class p2dQuality: public AlignmentQuality
{
public:

    p2dQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity()) : AlignmentQuality(src, ref, par, Toffset){cout<<"p2d quality"<<endl;}

    ~p2dQuality(){}

    std::vector<double> GetResiduals(){return {0,0,0}; }

    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};
/************** P2D ********************/



/************** CorAl *******************/

class CorAl: public AlignmentQuality
{
public:

    CorAl(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity())  : AlignmentQuality(src, ref, par, Toffset){}

    ~CorAl(){}

    std::vector<double> GetResiduals(){ return {0,0,0}; }
    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

class CFEARQuality: public AlignmentQuality
{
public:

    CFEARQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

    ~CFEARQuality(){}

    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};

class CorAlCartQuality: public AlignmentQuality
{
public:

    CorAlCartQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

    ~CorAlCartQuality(){}


};

class CorAlRadarQuality: public AlignmentQuality
{
public:

    CorAlRadarQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset = Eigen::Affine3d::Identity());

    ~CorAlRadarQuality(){}

    //std::vector<double> GetQualityMeasure(){return {sep_, joint_, 0};}

protected:

    void GetNearby(const pcl::PointXY& query, Eigen::MatrixXd& nearby_src, Eigen::MatrixXd& nearby_ref, Eigen::MatrixXd& merged);

    bool Covariance(Eigen::MatrixXd& x, Eigen::Matrix2d& cov, Eigen::Vector2d& mean);

    bool ComputeKLDiv(const Eigen::Vector2d& u0, const Eigen::Vector2d& u1, const Eigen::Matrix2d& S0, const Eigen::Matrix2d& S1, const int index);

    bool ComputeEntropy(const Eigen::Matrix2d& cov_sep, const Eigen::Matrix2d& cov_joint, int index);

    pcl::PointCloud<pcl::PointXY>::Ptr ref_pcd, src_pcd;

    pcl::KdTreeFLANN<pcl::PointXY> kd_src, kd_ref;

    std::vector<Eigen::Matrix2d> covs_sep_, cov_joint_;
    std::vector<Eigen::Vector2d> means_sep_, means_joint_;

    std::vector<double> sep_res_; // length ref + src
    std::vector<double> sep_intensity_;
    std::vector<bool> sep_valid; // length  ref + src
    std::vector<double> joint_res_; // length ref + src
    std::vector<double> diff_res_; // length ref + src
    double sep_ = 0, joint_ = 0, diff_ = 0, w_sum_ = 0;
    int count_valid = 0;
    double overlap_ = 0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ref_pcd_entropy, src_pcd_entropy, merged_entropy;

    int overlap_req_ = 1;





    // static CreateQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src);
};








class AlignmentQualityFactory
{
public:
    static AlignmentQuality_S CreateQualityType(std::shared_ptr<PoseScan>& ref, std::shared_ptr<PoseScan>& src,  const AlignmentQuality::parameters& pars, const Eigen::Affine3d& Toffset = Eigen::Affine3d::Identity()) {
        AlignmentQuality_S quality = nullptr;

        // CFEAR FEATURES + ANY SCORE (P2P/P2L/P2D)
        if(std::dynamic_pointer_cast<CFEARFeatures>(ref)!=nullptr && std::dynamic_pointer_cast<CFEARFeatures>(src)!=nullptr){
            quality = std::make_shared<CFEARQuality>(CFEARQuality(ref,src,pars,Toffset));
        }// RAW LIDAR (P2P/P2D/CORAL)
        if(std::dynamic_pointer_cast<BFARScan>(ref)!=nullptr && std::dynamic_pointer_cast<BFARScan>(src)!=nullptr){

            if(pars.method=="P2P")
                quality = AlignmentQuality_S(new p2pQuality(ref,src,pars,Toffset));
            if(pars.method=="keypoint_repetability")
                quality = AlignmentQuality_S(new keypointRepetability(ref,src,pars,Toffset));

        }// RAW LIDAR (P2P/P2D/CORAL)
        else if(std::dynamic_pointer_cast<RawLidar>(ref)!=nullptr && std::dynamic_pointer_cast<RawLidar>(src)!=nullptr){
            if(pars.method=="Coral")
                quality = std::make_shared<CorAl>(CorAl(ref,src,pars,Toffset));
            else if(pars.method=="P2D")
                quality = AlignmentQuality_S(new p2dQuality(ref,src,pars,Toffset));
            else if(pars.method=="P2P")
                quality = AlignmentQuality_S(new p2pQuality(ref,src,pars,Toffset));
        }
        else if(std::dynamic_pointer_cast<kstrongRadar>(ref)!=nullptr && std::dynamic_pointer_cast<kstrongRadar>(src)!=nullptr){
            if(pars.method=="Coral")
                quality = std::make_shared<CorAlRadarQuality>(CorAlRadarQuality(ref,src,pars,Toffset));
        }
        else if(std::dynamic_pointer_cast<kstrongStructuredRadar>(ref)!=nullptr && std::dynamic_pointer_cast<kstrongStructuredRadar>(src)!=nullptr){
            if(pars.method=="Coral")
                quality = std::make_shared<CorAlRadarQuality>(CorAlRadarQuality(ref,src,pars,Toffset));
            else if(pars.method=="P2P")
                quality = AlignmentQuality_S(new p2pQuality(ref,src,pars,Toffset));
        }
        else if(std::dynamic_pointer_cast<Cen2018Radar>(ref)!=nullptr && std::dynamic_pointer_cast<Cen2018Radar>(src)!=nullptr){
            if(pars.method=="P2P")
                quality = AlignmentQuality_S(new p2pQuality(ref,src,pars,Toffset));
        }
        else if(std::dynamic_pointer_cast<Cen2019Radar>(ref)!=nullptr && std::dynamic_pointer_cast<Cen2019Radar>(src)!=nullptr){

        }
        else if(std::dynamic_pointer_cast<CartesianRadar>(ref)!=nullptr && std::dynamic_pointer_cast<CartesianRadar>(src)!=nullptr){
            quality = std::make_shared<CorAlCartQuality>(CorAlCartQuality(ref,src,pars,Toffset));// RAW LIDAR (P2P/P2D/CORAL)
        }
        if(quality == nullptr){
            std::cerr<<"no quality metric for scan typee"<<endl;
            exit(0);
        }
        return quality;
    }
};

class AlignmentQualityPlot
{
public:
    AlignmentQualityPlot() {}

    static void PublishPoseScan(const std::string& topic, std::shared_ptr<PoseScan>& scan_plot, const Eigen::Affine3d& T, const std::string& frame_id, const int value=0);

    static void PublishCloud(const std::string& topic, pcl::PointCloud<pcl::PointXYZI>::Ptr& cld_plot, const Eigen::Affine3d& T, const std::string& frame_id, const int value = 0);

    static void PublishRadar(const std::string& topic, cv_bridge::CvImagePtr& img, const Eigen::Affine3d& T = Eigen::Affine3d::Identity(), const std::string& frame_id="", const ros::Time& t = ros::Time::now() );

    static void PublishTransform(const Eigen::Affine3d& T, const std::string& frame_id, const ros::Time& t = ros::Time::now());

    static std::map<std::string, ros::Publisher> pubs;


};


class AlignmentQualityInterface
{
public:
    AlignmentQualityInterface() {}
    static std::vector<bool> TrainingDataService(PoseScan_S& scan_current, PoseScan_S& scan_loop);
    static double AlignmentDataService(PoseScan_S& ref, PoseScan_S& strchr);
    static void UpdateTrainingData(PoseScan_S& ref, PoseScan_S& src, const bool visualize = false );
    static void SaveTrainingData(const std::string& training_data_sequence);
    static const std::vector<std::vector<double>> CreatePerturbations();
    static ros::Publisher pub_train_data;
    static std::vector<std::vector<double>> training_data_;
    static std::vector<std::vector<double>> vek_perturbation_;
private:
};

}
