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

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/common/transforms.h"

//cfear_radarodometry
#include "cfear_radarodometry/radar_filters.h"
#include "cfear_radarodometry/pointnormal.h"
#include "pcl/point_types.h"
#include "cfear_radarodometry/utils.h"
#include "string"

#include "alignment_checker/Utils.h"


//represent all possible scan types here. e.g. polar radar e.g. polarRadarKstrong CartesianRadar etc.


namespace CorAlignment{

using std::endl;
using std::cout;
using std::cerr;


typedef enum ScanType{none, rawlidar, rawradar, kstrong, kstrongStructured, kstrongCart, cfear, cen2018, cen2019, bfar}scan_type;

std::string Scan2str(const scan_type& val);

scan_type Str2Scan(const std::string& val);


class PoseScan
{
public:

    class Parameters
    {
    public:


        ScanType scan_type = rawlidar;
        double sensor_min_distance = 2.5;

        //Radar
        double range_res = 0.04328;

        //kstrong
        int kstrong = 12;
        double z_min = 80;
        //BFAR
        int window_size_ = 12;
        float offset_factor_= 30;
        float scale_factor = 1.0;
    
        //CFEAR
        double resolution = 3;
        bool compensate = true;
        bool ccw = false;
        bool normalize_intensity = false;

        float cart_resolution = 0.2384;
        int cart_pixel_width = 300;

        static const std::vector<std::string> HeaderToString(){
            return {"scan_type","sensor_min_distance","range_res","kstrong","z_min","compensate","ccw","cart_resolution","cart_pixel_width"};
        }
        const std::vector<std::string> ValsToString() const{
            return {Scan2str(scan_type) , std::to_string(sensor_min_distance) , std::to_string(range_res) , std::to_string(kstrong) , std::to_string(z_min),
                        std::to_string(compensate) , std::to_string(ccw) , std::to_string(cart_resolution) , std::to_string(cart_pixel_width)};
        }
        Parameters() {}
        std::string ToString(){
            std::ostringstream stringStream;
            const std::vector<std::string> header = HeaderToString();
            const std::vector<std::string> values = ValsToString();
            if(header.size() != values.size())
                throw std::runtime_error("size error ToString");
            for(int i = 0 ; i<header.size() ; i++)
                stringStream << header[i] << "\t" << values[i] <<endl;
            return stringStream.str();
        }

    };
    // PoseScan declarations
    static int pose_count;

    const int pose_id;
    Eigen::Affine3d Test_;
    Eigen::Affine3d Tmotion_; // the motion
    const PoseScan::Parameters pars_;


    PoseScan(const PoseScan::Parameters pars, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);
    PoseScan(const PoseScan::Parameters pars, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

    virtual ~PoseScan() {}

    const Eigen::Affine3d& GetAffine() {return Test_;}

    const std::string ToString(){return "PoseScan";}

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudCopy(const Eigen::Affine3d& T);

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudNoCopy() {return cloud_;}




protected:

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

typedef std::shared_ptr<PoseScan> PoseScan_S;

PoseScan_S RadarPoseScanFactory(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr radar_msg, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);


class RawRadar: public PoseScan{
public:

    RawRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);
    RawRadar(const PoseScan::Parameters& pars, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

    const std::string ToString(){return "RawRadar";}

    cv_bridge::CvImagePtr polar_;
    double range_res_;
};

class Cen2018Radar: public RawRadar{

public:
    Cen2018Radar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);
    cv::Mat f_polar_;
    Eigen::MatrixXd targets_;
};


class Cen2019Radar: public RawRadar{

public:
    Cen2019Radar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);
    cv::Mat f_polar_;
    Eigen::MatrixXd targets_;
};


class CartesianRadar: public RawRadar{
public:

    CartesianRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

    const std::string ToString(){return "CartesianRadar";}

    cv_bridge::CvImagePtr polar_, polar_filtered_, cart_;
    double  sensor_min_distance, cart_resolution_;
    int cart_pixel_width_;


};

class kstrongRadar: public RawRadar
{
public:

    kstrongRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion );
    kstrongRadar(const PoseScan::Parameters& pars, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion );

    const std::string ToString(){return "kstrongRadar";}

};


class kstrongStructuredRadar: public RawRadar
{
public:

    kstrongStructuredRadar(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion , bool peaks = true);
    kstrongStructuredRadar(const PoseScan::Parameters& pars, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);

    const std::string ToString(){return "structuredKstrongRadar";}

    pcl::PointCloud<pcl::PointXYZI>::Ptr kstrong_filtered_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr kstrong_peaks_;


};

class BFARScan: public RawRadar
{
public:

    BFARScan(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion );

    const std::string ToString(){return "BFAR_Radar";}

};

class CFEARFeatures: public kstrongRadar
{
public:

    CFEARFeatures(const PoseScan::Parameters& pars, cv_bridge::CvImagePtr& polar, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion);
    CFEARFeatures(const PoseScan::Parameters& pars, const CFEAR_Radarodometry::MapNormalPtr& CFEARFeatures, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion );
    const std::string ToString(){return "CFEARFeatures";}

    CFEAR_Radarodometry::MapNormalPtr CFEARFeatures_;
};

class RawLidar: public PoseScan{
public:

    RawLidar(const PoseScan::Parameters pars, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Affine3d& T, const Eigen::Affine3d& Tmotion ) : PoseScan(pars,T,Tmotion){ cloud_ = cloud; }

    const std::string ToString(){return "RawLidar";}

};


}

