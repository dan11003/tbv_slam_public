#pragma once

#include "stdio.h"
#include "vector"

#include <sstream>      // std::stringstream

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

//Coral includes
#include "alignment_checker/AlignmentQuality.h"
#include "alignment_checker/DataHandler.h"
#include "alignment_checker/ScanType.h"
#include "alignment_checker/Utils.h"

namespace CorAlignment {


class datapoint
{
public:

    double distance_;
    std::vector<double> residuals_;
    std::vector<double> perturbation_={0,0,0};
    std::vector<double> score_={0,0,0};
    int index_, ref_id_, src_id_;


    datapoint(const int index, const std::vector<double>& residuals,const std::vector<double>& perturbation, const std::vector<double>& score, std::shared_ptr<PoseScan>& ref, std::shared_ptr<PoseScan>& src) : residuals_(residuals), perturbation_(perturbation), score_(score), index_(index) {
        distance_ = (ref->GetAffine().translation()-src->GetAffine().translation()).norm();
        src_id_ = src->pose_id;
        ref_id_ = ref->pose_id;
    }

    static std::vector<std::string> HeaderToString(){
        return {"index" , "ref_id" , "src_id", "distance" ," score1" , "score2" , "score3" ,"aligned" , "error x" , "error y" , "error theta"};
    }

    const std::vector<std::string> ValsToString(){
        return { std::to_string(index_) , std::to_string(ref_id_) , std::to_string(src_id_) , std::to_string(distance_) ,
                    std::to_string(score_[0]) , std::to_string(score_[1]) , std::to_string(score_[2]) , std::to_string(aligned()) ,
                    std::to_string(perturbation_[0]) , std::to_string(perturbation_[1]) , std::to_string(perturbation_[2])};
    }

    bool aligned(){return datapoint::aligned(perturbation_);}

    static bool aligned(const std::vector<double>& perturbation);

};
class scanEvaluator
{
public:


    class parameters
    {
    public:
        parameters() {}

        int scan_spacing = 1;
        double scan_spacing_distance = -1.0;
        int rosbag_offset = 0;

        // offset parameters
        // For inducing cartesian position error y=r*sin(t) t depends on theta range (coverage) and offset_rotation_steps (resolution)
        // Ideally, theta_range is 360 deg and theta_range -> infinity, however we can approximate this by looking at a 1/4 of the full sweep e.g. in offset_rotation_steps=2steps
        double range_error = 0.5;
        double theta_range = 2*M_PI/4.0;
        double frame_delay = 0.0;


        int offset_rotation_steps = 2;

        //For inducing rotation error
        double theta_error = 0.57*M_PI/180.0;

        std::string output_directory = "";
        std::string output_meta_file = "params.txt";
        std::string output_eval_file = "eval.txt";
        std::string bag_file_path = "";

        std::string output_residual_prefix = "residuals_";
        std::string eval_name ="noname";
        std::string dataset = "", sequence = "";
        std::string input_odom_topic ="/gt";

        //Visualization
        bool visualize = true;

        static const std::vector<std::string> HeaderToString(){
            return {"output_directory","output_meta_file","output_eval_file","bag_file_path","eval_name","dataset","input_odom_topic","scan_spacing","scan_spacing_distance","rosbag_offset","range_error","theta_range","offset_rotation_steps","theta_error"};
        }

        const std::vector<std::string> ValsToString() const{
            return {output_directory , output_meta_file , output_eval_file , bag_file_path , eval_name , dataset ,
                        input_odom_topic , std::to_string(scan_spacing) , std::to_string(scan_spacing_distance) , std::to_string(rosbag_offset) , std::to_string(range_error) ,std::to_string(theta_range) , std::to_string(offset_rotation_steps) , std::to_string(theta_error)};
        }

        std::string ToString(){
            std::ostringstream stringStream;
            const std::vector<std::string> header = HeaderToString();
            const std::vector<std::string> values = ValsToString();
            if(header.size() != values.size())
                throw std::runtime_error("size error ToString");
            for(int i = 0 ; i<header.size() ; i++)
                stringStream << header[i] << "\t" << values[i]<<endl;;
            return stringStream.str();
        }

    };
    scanEvaluator( dataHandler_U& reader, const parameters& eval_par, const AlignmentQuality::parameters& alignment_par, const PoseScan::Parameters& scan_pars);

    void CreatePerturbations();

    void SaveEvaluation();



private:

    void InputSanityCheck();

    const parameters par_;
    const AlignmentQuality::parameters quality_par_;
    const PoseScan::Parameters scan_pars_;
    //dataHandler_U reader_;
    dataHandler_U reader_;

    std::vector< std::vector<double> > vek_perturbation_;
    std::vector<datapoint> datapoints_;
    ros::Publisher pub_train_data;
    ros::NodeHandle nh_;

};



}
