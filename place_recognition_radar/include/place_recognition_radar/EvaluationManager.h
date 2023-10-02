#pragma once

#include "place_recognition_radar/RadarScancontext.h"

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "vector"
#include "cfear_radarodometry/utils.h"

namespace PlaceRecognitionRadar
{

class EvaluationManager
{
public:
  /* PARAMETERS */

  struct Parameters
  {
    std::string node_name;
    std::string sequence = "";
    std::string dataset = "";
    std::string experiment_name;
    int nr_sc_thresholds;
    std::vector<double> distance_thresholds;
    bool save_results;
    std::string loop_eval_output_dir;

    const std::string namesToString()const
    {
      std::ostringstream stringStream;
      stringStream << "dataset,sequence";
      return stringStream.str();
    }

    const std::string valuesToString() const
    {
      std::ostringstream stringStream;
      stringStream << this->dataset << "," << this->sequence;
      return stringStream.str();
    }
  };

  struct LoopCandidates{
    Eigen::Affine3d Tfrom, Tto, Tclosest;
    double closest_loop_distance;
    double candidate_loop_distance;
    unsigned int id_from;
    unsigned int id_to;
    unsigned int id_close;
    int guess_nr;
    std::map<std::string,double> quality;
    Eigen::Affine3d Tgt_diff;


    std::string namesToString(){
      const std::string quality_str = Join(quality,true);
      return "from.x,from.y,from.z,to.x,to.y,to.z,close.x,close.y,close.z,diff.x,diff.y,diff.z,closest_loop_distance,candidate_loop_distance,id_from,id_to,id_close,guess_nr," + Join(quality,true);
    }

    std::string valuesToString(){
      std::ostringstream ss;
      ss << std::setprecision(6);
      //ss.setf(std::ios_base::fixed);
      const double x = Tgt_diff.translation()[0];
      const double y = Tgt_diff.translation()[1];
      const double theta = Tgt_diff.rotation().eulerAngles(0,1,2)[2];

      for(int i=0;i<3;i++)
        ss << Tfrom.translation()[i]<<",";
      for(int i=0;i<3;i++)
        ss << Tto.translation()[i]<<",";
      for(int i=0;i<3;i++)
        ss << Tclosest.translation()[i]<<",";

      ss << x << "," << y << "," << theta << ",";
      ss << closest_loop_distance << "," << candidate_loop_distance <<","<< id_from << "," << id_to << "," << id_close << "," << guess_nr << ","  << Join(quality,false);

      return ss.str();
    }
  };

  EvaluationManager(const EvaluationManager::Parameters& pars) : par(pars){}

  void updateResults(const Eigen::Affine3d& Tfrom, const Eigen::Affine3d& Tto, const Eigen::Affine3d& Tclosest, const Eigen::Affine3d& Tgt_diff, const double closest_loop_distance, const double candidate_loop_distance, const std::map<std::string,double>& quality, unsigned int id_from = 0, unsigned int id_to = 0, unsigned int id_close = 0, int guess_nr = 1);
  std::pair<bool,bool> getCandidateLoopStatus();
  bool writeResultsToCSV(const std::string& pars_str, const std::string& vals_str);

  std::vector<LoopCandidates> candidates_vek_;

  Parameters par;
private:

};


}
