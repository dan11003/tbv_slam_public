#include "place_recognition_radar/EvaluationManager.h"

namespace PlaceRecognitionRadar
{

void EvaluationManager::updateResults(const Eigen::Affine3d& Tfrom, const Eigen::Affine3d& Tto, const Eigen::Affine3d& Tclosest, const Eigen::Affine3d& Tgt_diff, const double closest_loop_distance, const double candidate_loop_distance, const std::map<std::string,double>& quality, unsigned int id_from , unsigned int id_to , unsigned int id_close,  int guess_nr)
{
  LoopCandidates l = {Tfrom, Tto, Tclosest , closest_loop_distance, candidate_loop_distance, id_from, id_to, id_close, guess_nr, quality, Tgt_diff};
  candidates_vek_.push_back(l);
}

std::pair<bool,bool> EvaluationManager::getCandidateLoopStatus()
{
  const double max_distance_ = 6;
  const double max_registration_translation_ = 4;
  const double max_registration_rotation_ = 2.5;

  const bool is_loop = this->candidates_vek_.back().closest_loop_distance < max_distance_;

  const double candidate_transl_error = this->candidates_vek_.back().Tgt_diff.translation().norm();
  const double candidate_rot_error = 180.0 / M_PI * std::fabs(this->candidates_vek_.back().Tgt_diff.rotation().eulerAngles(0,1,2)[2]);
  
  const bool candidate_close = (candidate_transl_error < max_registration_translation_) && (candidate_rot_error < max_registration_rotation_);
  const bool prediction_pos_ok = !is_loop || candidate_close;

  return std::pair<bool, bool>(is_loop, prediction_pos_ok);
}

bool EvaluationManager::writeResultsToCSV(const std::string& pars_str, const std::string& vals_str)
{
  if(candidates_vek_.empty()){
    cout << "EvaluationManager::writeResultsToCSV - nothing to save" << endl;
    return false;
  }
  // Path to evaluation folder

  if (!boost::filesystem::exists(par.loop_eval_output_dir))// Create subdirectory if it doesnt exists
    boost::filesystem::create_directory(par.loop_eval_output_dir);

  // Path to csv file
  const std::string csv_path = par.loop_eval_output_dir +"/" + "loop.csv";
  cout << "Writing result to: " << csv_path << endl;

  // Write results to csv file
  std::ofstream result_file(csv_path);

  // All parameter names (csv header)
  result_file << candidates_vek_.front().namesToString() <<"," << pars_str << "\n";

  // Parameter values
  for(auto && candidate : candidates_vek_)
    result_file << candidate.valuesToString() << "," << vals_str << std::endl;

  result_file.close();

  return true;
}


}
