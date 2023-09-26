#include "tbv_slam/loopclosure.h"

#include <boost/numeric/conversion/converter.hpp>
namespace tbv_slam{

void loopclosure::BaseParameters::GetParametersFromRos(ros::NodeHandle& nh){
  /*nh.param<double>("min_p", p_min, 0.5);
  nh.param<double>("rel_dist_sigma", rel_dist_sigma, 0.05);
  nh.param<double>("max_distance_close", DGTVP.max_d_close_, 5.0);
  nh.param<double>("min_distance_travel", DGTVP.min_d_travel_, 0.0);
  nh.param<double>("max_distance_travel", DGTVP.max_d_travel_, 2.0);ining*/
  //nh.param<std::string>("loop_closure_server_topic", loop_client_topic, "loop_closure");
}


void loopclosure::LoopClosureThread(){

  ros::Time tlast = ros::Time::now();
  ros::Rate r(50);

  while (ros::ok()) {
    r.sleep();
    ProcessFrame();
  }
}
bool loopclosure::ProcessFrame(){
  return SearchAndAddConstraint();
}
bool loopclosure::Register(const unsigned int from, const unsigned int to, const Eigen::Affine3d& Tfrom, const Eigen::Affine3d& Tto, Eigen::Affine3d& Talign, Eigen::Matrix<double, 6, 6> &reg_cov){
  // Build registration vectors
  auto normals_from = (*graph_)[from].cloud_normal_;
  auto normals_to = (*graph_)[to].cloud_normal_;
  std::vector<CFEAR_Radarodometry::MapNormalPtr> scans_vek{ normals_to, normals_from};
  std::vector<Eigen::Affine3d> T_vek{Tto, Tfrom};
  std::vector<Matrix6d> cov_vek{CFEAR_Radarodometry::Matrix6d::Identity(), CFEAR_Radarodometry::Matrix6d::Identity()};
  Covariance cov_sampled;

  // Ground truth - debug only
  const Eigen::Affine3d Tgt_from = PoseCeresToEig((*graph_)[from].Tgt);
  const Eigen::Affine3d Tgt_to = PoseCeresToEig((*graph_)[to].Tgt);
  const Eigen::Affine3d Tgt_diff = Tgt_to.inverse()*Tgt_from; // to -> from, for visualization of ground truth


  /*cout << "from: " << from << endl;
  cout << "to  : " << to << endl;
  cout << "reg src size: " << scans_vek.back()->GetCells().size() << endl;
  cout << "reg tar size: " << scans_vek.front()->GetCells().size() << endl;*/


  n_scan_normal_reg radar_reg(Str2Cost("P2L"));
  radar_reg.SetParameters(4,10);

  const bool reg_success = radar_reg.Register(scans_vek, T_vek, cov_vek, false); // Register loop and current
  const Eigen::Affine3d Trevised = T_vek.back();

  // if the sampling process succeeds, replace the covariance
  if(par_.use_covariance_sampling_in_loop_closure) {
    cout << "sampling covariance" << endl;
    ros::Time tc1 = ros::Time::now();
    if (approximateCovarianceBySampling(radar_reg, scans_vek, T_vek, cov_sampled)) {
      cov_vek.back() = cov_sampled;
    }
    ros::Time tc2 = ros::Time::now();
    cout << "time: " << tc2 - tc1 << endl;
  }
  else{
  cout << "ceres covariance" << endl;
  }


  // Visualize
  auto cloud_from = (*graph_)[from].cloud_nopeaks_;
  auto cloud_to = (*graph_)[to].cloud_nopeaks_;
  if(par_.visualize){
    MapPointNormal::PublishMap("map_point_normal_target", normals_to, Tto, "world", 45);// TARGET, loop (blue 45)
    CorAlignment::AlignmentQualityPlot::PublishCloud("registration/target", cloud_to, Tto, "target");

    MapPointNormal::PublishMap("map_point_normal_source", normals_from, Tfrom, "world", 192);// SOURCE, current (red 192)
    CorAlignment::AlignmentQualityPlot::PublishCloud("registration/source", cloud_from, Tfrom, "source");

    MapPointNormal::PublishMap("map_point_normal_source_revised", scans_vek.back(), Trevised, "world", 48);// SOURCE REVISED (green 48)
    CorAlignment::AlignmentQualityPlot::PublishCloud("registration/source_revised", cloud_from, Trevised, "source_revised");

    PoseGraphVis::pubTFForPose({Tto, Tfrom, Trevised, Tto*Tgt_diff}, {"target", "source", "source_revised", "source_gt"}, ros::Time::now());
  }


  if(reg_success){ // remove this during evaluation
    Talign = Trevised.inverse()*Tto;
    reg_cov = cov_vek.back();
    reg_cov.block<3,3>(0,0) = Trevised.inverse().rotation()*reg_cov.block<3,3>(0,0)*Trevised.inverse().rotation().transpose(); //Change frame to Tprev
  }

  return reg_success;
}

bool loopclosure::approximateCovarianceBySampling(n_scan_normal_reg &radar_reg,
                                                  std::vector<CFEAR_Radarodometry::MapNormalPtr> &scans_vek,
                                                  const std::vector<Eigen::Affine3d> &T_vek,
                                                  Covariance &cov_sampled){
  bool cov_sampled_success = true;

  std::vector<Eigen::Affine3d> T_vek_copy(T_vek);
  Eigen::Affine3d T_best_guess = T_vek_copy.back();

  double xy_sample_range = 0.2; // sampling in x and y axis around the estimated pose
  double theta_range = 0.0022;  // sampling on the yaw axis plus minus
  unsigned int number_of_steps_per_axis = 3;
  unsigned int number_of_samples = number_of_steps_per_axis*number_of_steps_per_axis*number_of_steps_per_axis;
  double cov_sampling_covariance_scaler = 4.0;

  Eigen::Affine3d sample_T = Eigen::Affine3d::Identity();
  double sample_cost = 0;  // return of the getCost function
  std::vector<double> residuals; // return of the getCost function
  Eigen::VectorXd samples_x_values(number_of_samples);
  Eigen::VectorXd samples_y_values(number_of_samples);
  Eigen::VectorXd samples_yaw_values(number_of_samples);
  Eigen::VectorXd samples_cost_values(number_of_samples);

  std::vector<double> xy_samples = linspace<double>(-xy_sample_range, xy_sample_range, number_of_steps_per_axis);
  std::vector<double> theta_samples = linspace<double>(-theta_range, theta_range, number_of_steps_per_axis);

  // Sample the cost function according to the settings, optionally dump the samples into a file
  int vector_pointer = 0;
  for (int theta_sample_id = 0; theta_sample_id < number_of_steps_per_axis; theta_sample_id++) {
    for (int x_sample_id = 0; x_sample_id < number_of_steps_per_axis; x_sample_id++) {
      for (int y_sample_id = 0; y_sample_id < number_of_steps_per_axis; y_sample_id++) {

        sample_T.translation() = Eigen::Vector3d(xy_samples[x_sample_id],
                                                 xy_samples[y_sample_id],
                                                 0.0) + T_best_guess.translation();
        sample_T.linear() = Eigen::AngleAxisd(theta_samples[theta_sample_id],
                                              Eigen::Vector3d(0.0, 0.0, 1.0)) * T_best_guess.linear();

        T_vek_copy.back() = sample_T;
        radar_reg.GetCost(scans_vek, T_vek_copy, sample_cost, residuals);

        samples_x_values[vector_pointer] = xy_samples[x_sample_id];
        samples_y_values[vector_pointer] = xy_samples[y_sample_id];
        samples_yaw_values[vector_pointer] = theta_samples[theta_sample_id];
        samples_cost_values[vector_pointer] = sample_cost;
        vector_pointer ++;

      }
    }
  }

  //Find the approximating quadratic function by linear least squares
  // f(x,y,z) = ax^2 + by^2 + cz^2 + dxy + eyz + fzx + gx+ hy + iz + j
  // A = [x^2, y^2, z^2, xy, yz, zx, x, y, z, 1]
  Eigen::MatrixXd A(number_of_samples, 10);
  A << samples_x_values.array().square().matrix(),
      samples_y_values.array().square().matrix(),
      samples_yaw_values.array().square().matrix(),
      (samples_x_values.array()*samples_y_values.array()).matrix(),
      (samples_y_values.array()*samples_yaw_values.array()).matrix(),
      (samples_yaw_values.array()*samples_x_values.array()).matrix(),
      samples_x_values,
      samples_y_values,
      samples_yaw_values,
      Eigen::MatrixXd::Ones(number_of_samples,1);

  Eigen::VectorXd quad_func_coefs = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(samples_cost_values);

  // With the coefficients, we can contruct the Hessian matrix (constant for a given function)
  Eigen::Matrix3d hessian_matrix;
  hessian_matrix << 2*quad_func_coefs[0],   quad_func_coefs[3],   quad_func_coefs[5],
      quad_func_coefs[3], 2*quad_func_coefs[1],   quad_func_coefs[4],
      quad_func_coefs[5],   quad_func_coefs[4], 2*quad_func_coefs[2];

  // We need to check if the approximating function is convex (all eigs positive, we don't went the zero eigs either)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(hessian_matrix);
  Eigen::Vector3d eigValues;
  Eigen::Matrix3d covariance_3x3;

  if (eigensolver.info() != Eigen::Success) {
    cov_sampled_success = false;
    std::cout << "Covariance sampling warning: Eigenvalues search failed." << std::endl;
  }
  else{
    eigValues = eigensolver.eigenvalues();
    if(eigValues[0] <= 0.0 || eigValues[1] <= 0.0 || eigValues[2] <= 0.0){
      cov_sampled_success = false;
      std::cout << "Covariance sampling warning: Quadratic approximation not convex. Sampling will not be used for this scan." << std::endl;
    }
    else{
      // Compute the covariance from the hessian and scale by the score
      double score_scale = 1.0;
      if(radar_reg.GetCovarianceScaler(score_scale)) {

        covariance_3x3 = 2.0 * hessian_matrix.inverse() * score_scale * cov_sampling_covariance_scaler;

        //We need to construct the full 6DOF cov matrix
        cov_sampled = Eigen::Matrix<double, 6, 6>::Identity();
        cov_sampled.block<2, 2>(0, 0) = covariance_3x3.block<2, 2>(0, 0);
        cov_sampled(5, 5) = covariance_3x3(2, 2);
        cov_sampled(0, 5) = covariance_3x3(0, 2);
        cov_sampled(1, 5) = covariance_3x3(1, 2);
        cov_sampled(5, 0) = covariance_3x3(2, 0);
        cov_sampled(5, 1) = covariance_3x3(2, 1);
      }
      else cov_sampled_success = false;
    }
  }
  return cov_sampled_success;
}

Eigen::MatrixXd loopclosure::QualityToFeatures(std::map<string,double>& quality){
  const int features_size = par_.model_features.size();
  Eigen::MatrixXd X(1,features_size);
  int idx = 0;
  for(std::string feature : par_.model_features){
    X(0,idx++) = quality[feature];
  }
  return X;
}

double loopclosure::VerificationModel(std::map<string,double>& quality){
  Eigen::MatrixXd X = QualityToFeatures(quality);

  // If classifier is not fitted, use preset coef. FEATURES FOR (odom-bounds sc-sim alignment_quality)
  if(!verification_classifier_.IsFit()){
    Eigen::Matrix<double,1,3> coef;
    Eigen::Matrix<double,1,3> X_(X);
    coef << -2.89398535, -9.40230684, 0.23891265;
    const double bias = 2.67958289;
    const double z = coef*X_.transpose() + bias;
    const double p = 1.0/(1.0 + std::exp(-z));
    return p;
  }

  const double z =  verification_classifier_.predict_linear(X)(0);
  const double p = 1.0/(1.0 + std::exp(-z));
  return p;

}

void loopclosure::AddVerificationTrainingData(Constraint3d& constraint){
  if(par_.model_training_file_save != ""){
    std::pair<bool, bool> candiadte_status = graph_->eval_manager.getCandidateLoopStatus();   // Get status of candidate (check if ok to be used as training data)
    if(candiadte_status.second){
      Eigen::MatrixXd X = QualityToFeatures(constraint.quality);
      Eigen::VectorXd y(1);
      y << candiadte_status.first;
      cout << "Added training datapoint: y: " << y << ", X: " << X << endl;
      verification_classifier_.AddDataPoint(X, y);
    }
  }
}

void loopclosure::SaveVerificationTrainingData(){
  if(par_.model_training_file_save != ""){
    verification_classifier_.fit();
    const std::string training_data_path = ros::package::getPath("tbv_slam") + "/model_parameters/" + par_.model_training_file_save;
    verification_classifier_.SaveData(training_data_path);
  }
}

void loopclosure::ApplyConstratins(std::vector<std::pair<double,Constraint3d>>& candidate_probabilities){
  if(candidate_probabilities.empty())
    return;

  // Sort probabilities (large to small)
  std::sort(candidate_probabilities.begin(), candidate_probabilities.end(), [](const std::pair<double,Constraint3d>& c1, const std::pair<double,Constraint3d>& c2){
    return c1.first > c2.first;
  });

  // Selection strategy for candidate ( BEST (first in sorted vector) or ALL )
  auto itr_end = par_.all_candidates ? candidate_probabilities.end() : ++candidate_probabilities.begin();
  for(auto itr = candidate_probabilities.begin(); itr != itr_end; ++itr){
    if(itr->first > par_.model_threshold){
      graph_->AddConstraintThSafe(itr->second);
      graph_->AddConstraintThSafe(CreateCandidateConstraint(itr->second.id_begin, itr->second.id_end, "Verified candidate" )); // Visualization Only
      cout << "FOUND LOOP between: " << itr->second.id_begin << " and " << itr->second.id_end << endl;

      if(par_.visualize){
        // Point clouds (src and target)
        Eigen::Affine3d T_src = (*graph_)[itr->second.id_begin].GetPose();
        Eigen::Affine3d T_target = (*graph_)[itr->second.id_end].GetPose();
        Eigen::Affine3d T_target_reg = T_src * PoseCeresToEig(itr->second.t_be);

        auto cloud_src = (*graph_)[itr->second.id_begin].cloud_nopeaks_;
        auto cloud_target = (*graph_)[itr->second.id_end].cloud_nopeaks_;
        CorAlignment::AlignmentQualityPlot::PublishCloud("demo/cloud_src_demo", cloud_src, T_src , "demo_src");
        CorAlignment::AlignmentQualityPlot::PublishCloud("demo/cloud_target_demo", cloud_target, T_target_reg , "demo_target");

        // Loop constraint (line between source and target)
        static ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>("demo/loop_marker", 100);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "demo_maker";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.header.stamp = ros::Time::now();
        marker.id = itr->second.id_begin;
        
        geometry_msgs::Point point_src, point_target;
        tf::pointEigenToMsg(T_src.translation(), point_src);  // Set point position for src
        tf::pointEigenToMsg(T_target.translation(), point_target); // Set point position for target
        
        marker.points.push_back(point_src);
        marker.points.push_back(point_target);
        
        pub.publish(marker);  // Publish marker
      }

    }
  }
}

bool loopclosure::RegisterLoopCandidate(const unsigned int from, const unsigned int to, Constraint3d& constraint){
  if(par_.visualize){
    CorAlignment::AlignmentQualityPlot::PublishCloud("registration/verify_from", (*graph_)[from].cloud_nopeaks_, (*graph_)[from].GetPose() , "from");
    CorAlignment::AlignmentQualityPlot::PublishCloud("registration/verify_to", (*graph_)[to].cloud_nopeaks_, (*graph_)[to].GetPose() , "to");
  }

  if (par_.gt_loop) {
    cout << "gt loop" << endl;
    if( (*graph_)[from].has_Tgt_ && (*graph_)[from].has_Tgt_){
      Eigen::Affine3d Tdiff = PoseCeresToEig((*graph_)[from].Tgt).inverse()*PoseCeresToEig((*graph_)[to].Tgt);
      if(Tdiff.translation().norm() < 5){
        constraint = {from, to, Tdiff, 0.000001*Matrix6d::Identity(), ConstraintType::loop_appearance};
        return true;
      }
      else {
        return false;
      }
    }
    else
      return false;
  }else{

    Eigen::Affine3d T;
    //Eigen::Affine3d Tsrc = (type == loop_appearance) ?  (*graph_)[from].GetPose() : (*graph_)[from].GetPose()*PoseCeresToEig(constraint.t_be); // this correct?

    const Eigen::Affine3d Tfrom =   (*graph_)[from].GetPose(); // use guess
    const Eigen::Affine3d Tto = Tfrom*PoseCeresToEig(constraint.t_be);
    if(par_.visualize){
      CorAlignment::AlignmentQualityPlot::PublishCloud("registration/target_actual",  (*graph_)[to].cloud_nopeaks_, (*graph_)[to].GetPose(), "target_actual");
    }

    Eigen::Affine3d Tdiff = Eigen::Affine3d::Identity();
    Matrix6d Cov = Matrix6d::Identity();
    bool reg_ok = true;
    if(par_.registration_disabled){
      Tdiff = (*graph_)[from].GetPose().inverse()*(*graph_)[to].GetPose(); // not tested
    }else{
      reg_ok = Register(from, to, Tfrom, Tto, Tdiff, Cov);
    }

    constraint.t_be = PoseEigToCeres(Tdiff);
    constraint.information = Cov.inverse();
    return reg_ok;
  }
}
double loopclosure::VerifyLoopCandidate(Constraint3d& constraint){

  const Eigen::Affine3d Tfrom = (*graph_)[constraint.id_begin].GetPose();
  const Eigen::Affine3d Tto = Tfrom*PoseCeresToEig(constraint.t_be);
  const unsigned int from = constraint.id_begin;
  const unsigned int to  = constraint.id_end;

  ros::Time t0 = ros::Time::now();
  VerifyByAlignment(from, Tfrom, to, Tto, constraint.quality);
  ros::Time t1 = ros::Time::now();
  VerifyByOdometry(from, to, constraint.quality[ODOM_BOUNDS]);
  ros::Time t2 = ros::Time::now();
  const double verified_prob = par_.verification_disabled ? 0.0 : VerificationModel(constraint.quality);
  ros::Time t3 = ros::Time::now();
  CFEAR_Radarodometry::timing.Document("VerifyByAlignment", CFEAR_Radarodometry::ToMs(t1-t0));
  CFEAR_Radarodometry::timing.Document("VerifyByOdometry", CFEAR_Radarodometry::ToMs(t2-t1));
  CFEAR_Radarodometry::timing.Document("VerificationModel", CFEAR_Radarodometry::ToMs(t3-t2));
  return verified_prob;

}

double loopclosure::Distance(RadarScans::iterator& from, RadarScans::iterator& to){
  Pose3d p1 = (*from).second.T, p2 = (*to).second.T;
  double distance = (p2.p - p1.p).norm();
  return distance;
}


//Searches for nearby poses. creates pairs where the relative distance is at minima, one pair per origin pose.
bool GTVicinityClosure::SearchAndAddConstraint(){

  cout << "Started GTVicinityClosure: min_travel" << min_d_travel_ << ", max_travel:" << max_d_travel_ << ", max_d_close: " << max_d_close_ << endl;
  std::vector< std::pair<unsigned int, unsigned int> >  candidates_itrs;
  graph_->m_graph.lock();
  auto itr_begin =  graph_->GetPoses().begin();
  auto itr_end  =  graph_->GetPoses().end();

  unsigned int idx = 0;
  ConstraintsHandler ch(graph_->GetConstraints());

  for(; itr_current != itr_end ; itr_current++){
    cout << "frame: " << std::distance(itr_begin,itr_current) << "/" <<std::distance(itr_begin, itr_end) << endl;
    double closest_rel_distance = DBL_MAX;
    int idx_closest = -1;
    for(auto itr_to = std::next(itr_current); itr_to != itr_end && ros::ok() ; itr_to++){

      std::pair<unsigned int,unsigned int> key = std::make_pair(itr_current->first, itr_to->first);
      if(pair_attempted_.find(key) != pair_attempted_.end())
        continue;
      else
        pair_attempted_[key] = true;

      const double eucl_distance = graph_->EuclidianDistance(itr_current->first, itr_to->first);
      if( eucl_distance <= max_d_close_ ) // Nodes "APPEAR" to be nearby
      {
        //cout << "Nearby: " <<eucl_distance<<" - "<< itr_from->first << "," << itr_to->first << endl;
        const double trav_distance =  graph_->TraveledDistance(itr_current->first,itr_to->first );
        //cout << "traveled:: " << trav_distance << endl;
        if( min_d_travel_ <= trav_distance  && trav_distance <= max_d_travel_) // If the drift needs to be only slightly corrected  - Medium distance traveled
        {
          const double rel_distance = eucl_distance/trav_distance;
          if (rel_distance < closest_rel_distance) {
            idx_closest = itr_to->first;
            closest_rel_distance = rel_distance;
          }
        }
      }
    }
    if(idx_closest != -1){
      //cout << "Nearby: " <<eucl_distance<<" -   "<< itr_from->first << "," << itr_to->first << endl;
      //cout << "traveled:: " << trav_distance << endl;
      //cout << "add candidate: " << itr_from->first << "," << idx_closest << endl;
      candidates_itrs.push_back(std::make_pair(itr_current->first, idx_closest));
    }
  }
  graph_->m_graph.unlock();
  if(candidates_itrs.empty())
    cout << "Loop closure finished - nothing new to search" << endl;
  else
    cout << "Found " <<candidates_itrs.size() <<" new constraints" << endl;

  for(auto&& cand : candidates_itrs){
    if(!ros::ok())
      return false;
    //cout<<"Distance traveled: "<<DistanceTraveled(candidates_itrs[idx_min].first, candidates_itrs[idx_min].second )<<endl;
    //Add as candidate regardless
    const unsigned int from = std::max(cand.first, cand.second);
    const unsigned int to = std::min(cand.first, cand.second);
    Constraint3d Ccandidate = {from, to, Pose3d::Identity(), 0.01*Covariance::Identity(), Constrainttype::candidate};
    graph_->AddConstraintThSafe(Ccandidate);
    cout << "Evaluate: " << cand.first <<" - " << cand.second << endl;
    Constraint3d Cloop;
    RegisterLoopCandidate(from, to, Cloop);

    if(VerifyLoopCandidate(Cloop))
      graph_->AddConstraintThSafe(Cloop);
  }
  if(itr_current == itr_end)
    return false;
  else
    return true;
  cout << "Loop closure finished" << endl;
}

bool MiniClosure::SearchAndAddConstraint(){
  if (!MiniClosure_enabled) {
    return false;
  }

  //Eigen::MatrixXd dist = DistanceGraph();
  cout << "Started MiniClosure thread: min_travel" << min_d_travel_ << ", max_travel:" << max_d_travel_ << ", max_d_close: " << max_d_close_ << endl;
  //std::vector< std::pair<double,unsigned int>> d_candidates;
  std::vector< std::pair<unsigned int, unsigned int> >  candidates_itrs;
  graph_->m_graph.lock();

  unsigned int idx = 0;
  ConstraintsHandler ch(graph_->GetConstraints());
  const auto itr_begin = graph_->GetPoses().begin();
  const auto itr_end = graph_->GetPoses().end();

  for(auto itr_current = itr_begin ; itr_current != itr_end ; itr_current++){
    if(origin_attempted_.find(itr_current->first) != origin_attempted_.end()) // exists, means it's already searched
      continue;

    double closest_rel_distance = DBL_MAX;
    int idx_closest = -1;
    double trav_distance = 0;
    for(auto itr_to = std::next(itr_current); itr_to !=  itr_end && ros::ok() ; itr_to++){
      auto itr_prev = std::prev(itr_to);
      trav_distance += graph_->TraveledDistance(itr_prev->first,itr_to->first ); // accumulate

      if(trav_distance < min_d_travel_)
        continue;
      else if(trav_distance > max_d_travel_){
        origin_attempted_[itr_current->first] = true;
        break;
      }

      std::pair<unsigned int,unsigned int> key = std::make_pair(itr_current->first, itr_to->first);
      if(pair_attempted_.find(key) != pair_attempted_.end()) // Already considereds
        continue;
      else
        pair_attempted_[key] = true;

      const double eucl_distance = graph_->EuclidianDistance(itr_current->first, itr_to->first);
      if( eucl_distance <= max_d_close_ ){ // Nodes "APPEAR" to be nearby
        /*cout << "Nearby: " <<eucl_distance<<" - "<< itr_from->first << "," << itr_to->first << endl;
        cout << "traveled:: " << trav_distance << endl;
        cout << "rel_distance:: " << trav_distance << endl;*/
        const double rel_distance = eucl_distance/trav_distance;
        if (rel_distance < closest_rel_distance) {
          idx_closest = itr_to->first;
          closest_rel_distance = rel_distance;
        }
      }
    }
    if(idx_closest != -1){
      candidates_itrs.push_back(std::make_pair(itr_current->first, idx_closest));
    }
  }
  graph_->m_graph.unlock();
  if(candidates_itrs.empty())
    cout << "Mini Loop closure finished - nothing new to search" << endl;
  else
    cout << "Found " <<candidates_itrs.size() <<" new constraints" << endl;


  //std::sort(d_candidates.begin(), d_candidates.end() );
  for(auto&& cand : candidates_itrs){
    if(!ros::ok())
      return false;
    //cout<<"Distance traveled: "<<DistanceTraveled(candidates_itrs[idx_min].first, candidates_itrs[idx_min].second )<<endl;
    //Add as candidate regardless
    const unsigned int from = std::max(cand.first, cand.second);
    const unsigned int to = std::min(cand.first, cand.second);
    graph_->AddConstraintThSafe(CreateCandidateConstraint(from, to, "miniloop candidate"));
    cout << "Evaluate: " << cand.first <<" - " << cand.second << endl;
    Constraint3d Cminiloop = CreateMiniloopConstraint(from, to);

    if(VerifyLoopCandidate(Cminiloop)){
      graph_->AddConstraintThSafe(CreateCandidateConstraint(from, to, "miniloop verified"));
      graph_->AddConstraintThSafe(Cminiloop);
    }
  }
  if(itr_current == itr_end)
    return false;
  else return true;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ScanContextClosure::ScansToLocalMap(RadarScans::iterator& itr, const Eigen::Affine3d& T){

  pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
  const int nearby_elements = par_.DSCCP.N_aggregate ;
  for (int i = itr->second.idx_ - nearby_elements ; i <= itr->second.idx_ + nearby_elements && ros::ok() ; i++)  {
    if(graph_->NodeExists(i)){
      if(par_.DSCCP.use_peaks)
        pcl::transformPointCloud( *(*graph_)[i].cloud_peaks_, *tmp, (*graph_)[i].GetPose() );
      else
        pcl::transformPointCloud( *(*graph_)[i].cloud_nopeaks_, *tmp, (*graph_)[i].GetPose() );
      *merged += *tmp;
    }
  }
  pcl::transformPointCloud(*merged, *merged, itr->second.GetPose().inverse());
  return merged;
}

void ScanContextClosure::CreateContext(RadarScans::iterator& itr, Eigen::Affine3d& Todom){
  ros::Time t = ros::Time::now();
  if(par_.DSCCP.raw_scan_context){
    cv::Mat radar_scan_img;
    reader_->Get(itr->second.stamp_, radar_scan_img);
    rsc_.makeAndSaveScancontextAndKeysRadarRaw(radar_scan_img, Todom);
    //cout << "current: " << itr->second.idx_ << endl;
  }else{
    auto merged = ScansToLocalMap(itr, Todom);

    //if(par_.idx_halt != 0 && itr->second.idx_ >= par_.idx_halt  )
    //  rsc_.par.desc_plot = true;
    if(par_.visualize){
      CorAlignment::AlignmentQualityPlot::PublishCloud("loopclosure/context", merged, itr->second.GetPose(), "context");
    }
    rsc_.makeAndSaveScancontextAndKeysRadarCloud(merged, itr->second.GetPose());

  }
  ros::Time tend = ros::Time::now();
  //cout << "create context: " << tend - t << endl;
}

bool ScanContextClosure::SearchAndAddConstraint(){
  //cout << "bool ScanContextClosure::SearchAndAddConstraint(){" << endl;

  static ros::Publisher pub = nh_.advertise<nav_msgs::Odometry>("odometry_vis", 100);

  std::vector< std::pair<unsigned int, unsigned int> >  candidates_itrs;
  graph_->m_graph.lock();
  ConstraintsHandler ch(graph_->GetConstraints());
  if (!scan_iterator_initialized_) {
    if (graph_->GetPoses().size() > 0) {
      itr_current = std::next(graph_->GetPoses().begin(), 0 /*par_.dataset_start_offset*/);
      scan_iterator_initialized_ = true;
    } else {
      graph_->m_graph.unlock();
      return false;
    }
  }
  auto itr_begin = graph_->GetPoses().begin();
  auto itr_end = std::prev(graph_->GetPoses().end());
  if (!par_.threading) {
    itr_end = graph_->GetPoses().end(); // if no threading is used, prevent not using the last node
  }

  size_t count_itrs = 0;
  ros::Time t1 = ros::Time::now();
  graph_->m_graph.unlock();

  //cout << "Training: " << ros::Time::now() - t1 << endl;
  for( ; itr_current != itr_end && ros::ok() ; itr_current++){
    if(++count_itrs == 200){ // max 5 iterations at a time
      cout << "Pause SC loop" << endl;
      break;
    }
    ros::Time t0 = ros::Time::now();
    //std::cout << "Scan context: " << std::distance(itr_begin,itr_current) << "/" << std::distance(itr_begin,itr_end)-1 << std::endl;
    Eigen::Affine3d current_pose = itr_current->second.GetPose();
    //Eigen::Affine3d current_pose = std::next(itr_current)->second.GetPose();
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = t0;
    pose_msg.header.frame_id = "world";
    tf::poseEigenToMsg(current_pose, pose_msg.pose.pose);
    pub.publish(pose_msg);

    if(itr_current != itr_begin ){
      Todom_ = Todom_*ch.RelativeMotion(std::prev(itr_current)->second.idx_, itr_current->second.idx_);
    }
    if (origin_attempted_.find(itr_current->second.idx_) != origin_attempted_.end() || itr_begin == itr_end){
      continue;
    }
    //graph_->m_graph.lock();

    ros::Time t1 = ros::Time::now();
    CreateContext(itr_current, Todom_);
    ros::Time t2 = ros::Time::now();
    CFEAR_Radarodometry::timing.Document("Descriptor", CFEAR_Radarodometry::ToMs(t2-t1));

    const auto candidates = rsc_.detectLoopClosureID();
    ros::Time t3 = ros::Time::now();
    CFEAR_Radarodometry::timing.Document("Detect loop", CFEAR_Radarodometry::ToMs(t3-t2));

    if(candidates.empty()){
      graph_->UpdateStatistics(std::make_pair(itr_current->second.idx_,itr_current->second.idx_), Eigen::Affine3d::Identity(), {{ODOM_BOUNDS,1.0},{SC_SIM,1.0+par_.DSCCP.rsc_pars.odometry_coupled_closure},{COMBINED_COST,-20.0}}, -1); // Force classified as not a loop
    }
    std::vector<std::pair<double, Constraint3d>> candidate_probabilities;

    for(auto itr = candidates.begin() ;  itr != candidates.end() ; itr++  ){
      ros::Time t5 = ros::Time::now();
      unsigned int idx_from = itr_current->second.idx_; //+ par_.dataset_start_offset;
      unsigned int idx_to = itr->nn_idx; //+ par_.dataset_start_offset;



      // Visualize candidate "descriptor"
      Eigen::Affine3d T = (*graph_)[idx_to].GetPose();
      auto radar_itr = graph_->GetPoses().Get(idx_to);
      auto merged = ScansToLocalMap(radar_itr, T);
      if(par_.visualize){
        CorAlignment::AlignmentQualityPlot::PublishCloud("loopclosure/context_to", merged, T , "to");
      }
      if(par_.idx_halt != 0 && itr_current->second.idx_ >= par_.idx_halt  )
        char c = getchar();

      unsigned int guess_nr = std::distance(candidates.begin(),itr);
      const float sc_cand_yaw = itr->yaw_diff_rad;
      const float sc_cand_sim = itr->min_dist;
      auto trusted_candidate = std::make_pair(idx_from,idx_to);
      //cout  <<"sim: " <<  sc_cand_sim << endl;
      //cout  <<"yaw: " <<  sc_cand_yaw << endl;

      if(par_.DSCCP.rsc_pars.odometry_coupled_closure){
        if(par_.speedup && itr->min_dist_odom > 0.7 ){
          graph_->UpdateStatistics(std::make_pair(idx_from,idx_to), Eigen::Affine3d::Identity(), {{ODOM_BOUNDS,itr->min_dist_odom},{SC_SIM,itr->min_dist},{COMBINED_COST,-20}}, guess_nr); // Force classified as not a loop
          ros::Time t7 = ros::Time::now();
          CFEAR_Radarodometry::timing.Document("Loop-full", CFEAR_Radarodometry::ToMs(t7-t0));
          continue;
        }
      }

      // Create a "guess"
      const double src_yaw_offset = -sc_cand_yaw;

      const auto rot_offset = Eigen::AngleAxisd(-src_yaw_offset, Eigen::Vector3d::UnitZ());
      const Eigen::Affine3d transl_offset = (par_.transl_guess) ?  itr->Taug.inverse() : Eigen::Affine3d::Identity();
      const Eigen::Affine3d Tsrcguess =  transl_offset*rot_offset;

      if(par_.visualize){
        graph_->AddConstraintThSafe(CreateCandidateConstraint(idx_from, idx_to, "Trusted candidate")); // Visualization Only
      }
      Constraint3d ApperanceCandidate = CreateAppearanceConstraint(idx_from, idx_to, sc_cand_sim, Tsrcguess);

      ros::Time t6 = ros::Time::now();
      bool reg_ok = RegisterLoopCandidate(idx_from, idx_to, ApperanceCandidate);
      ros::Time t7 = ros::Time::now();
      CFEAR_Radarodometry::timing.Document("Register", CFEAR_Radarodometry::ToMs(t7-t6));

      const double prob = VerifyLoopCandidate(ApperanceCandidate);
      ros::Time t8 = ros::Time::now();
      CFEAR_Radarodometry::timing.Document("Verify loop candidate", CFEAR_Radarodometry::ToMs(t8-t7));

      candidate_probabilities.push_back(std::make_pair(prob, ApperanceCandidate));
      
      graph_->UpdateStatistics(trusted_candidate, PoseCeresToEig(ApperanceCandidate.t_be), ApperanceCandidate.quality, guess_nr); // add statistics to evaluate if at least one of the candidates is a real loop
      ros::Time t9 = ros::Time::now();
      CFEAR_Radarodometry::timing.Document("gather statistics", CFEAR_Radarodometry::ToMs(t9-t8));

      if (par_.model_training_file_save != "") {
        AddVerificationTrainingData(ApperanceCandidate);  // Add training data to verification_model_
      }
    }

    ros::Time t10 = ros::Time::now();
    ApplyConstratins(candidate_probabilities);  // Apply contraints depending on candidate probabilities and selected strategy
    ros::Time t11 = ros::Time::now();
    CFEAR_Radarodometry::timing.Document("Apply contraints", CFEAR_Radarodometry::ToMs(t11-t10));

    //graph_->m_graph.unlock();
    ros::Time t_end = ros::Time::now();

    CFEAR_Radarodometry::timing.Document("Loop-full", CFEAR_Radarodometry::ToMs(t_end-t0));
    //cout << "loop: " << tend - t << endl;
  }

  if(itr_current == itr_end){
    if (par_.model_training_file_save != "") {
      SaveVerificationTrainingData(); // Save training data from verification_model_
    }
    return false;
  }else{
    return true;
  }


}


inline Eigen::Matrix<double,6,6> ScanContextClosure::Score2Cov(double score){
  double V = 10*10*score+1;
  Eigen::Matrix<double,6,1> v;
  //v<<V,V,V,100*V,100*V,100*V;
  v << 100000, 100000, 100000, 1000000000, 1000000000, 1000000000;
  Eigen::Matrix<double,6,6> Cov = v.asDiagonal();
  return Cov;
}



void loopclosure::VerifyByAlignment(const unsigned int from, const Eigen::Affine3d& Tfrom_revised, const unsigned int to, const Eigen::Affine3d& Tfixed, std::map<string,double>& quality){
  ScanLearningInterface::s_scan sfrom = {Tfrom_revised, (*graph_)[from].cloud_nopeaks_, (*graph_)[from].cloud_peaks_, (*graph_)[from].cloud_normal_};
  ScanLearningInterface::s_scan sto   = {Tfixed,        (*graph_)[to].cloud_nopeaks_,   (*graph_)[to].cloud_peaks_,   (*graph_)[to].cloud_normal_};
  Eigen::MatrixXd xcoral, xcfear;
  bool valid = true;
  sli_.PredAlignment(sfrom, sto, quality, xcoral, xcfear, valid);

  //const double overlap = 2.0*xcfear(0,1)/(sfrom.CFEAR->GetSize() + sto.CFEAR->GetSize());

  //cout << xcfear << endl;
  /* if( overlap < 0.3){
    //cout << "overlap: " << overlap << endl;
    //cout << "LOOP INVALID" << endl;
    quality["CFEAR"] = 0;
    quality["coral"] = 0;
  }*/
}
void loopclosure::VerifyByOdometry(const unsigned int from, const unsigned int to, double& similarity){
  if(!par_.verify_via_odometry)
  {
    similarity = 1;
    return;
  }

  ConstraintsHandler ch(graph_->GetConstraints());
  Eigen::Affine3d Todom = Eigen::Affine3d::Identity();
  double odom_trav_distance = 0;
  for (unsigned int i = to ; i < from; i++) {
    Eigen::Affine3d Tdiff = ch.RelativeMotion(i,i+1);
    odom_trav_distance += Tdiff.translation().norm();
    Todom = Todom*Tdiff;
  }
  const double odom_est_distance = Todom.translation().norm();
  const double error = std::max(odom_est_distance - 5.0 ,0.0); // if within 5 meters, always considered nearby
  const double rel_error = error/odom_trav_distance;
  const double probabi1lity = exp(-rel_error*rel_error/(2*par_.odom_sigma_error*par_.odom_sigma_error));
  similarity = 1 - probabi1lity;
  /*cout << "odom_trav_distance: " << odom_trav_distance << endl;
  cout << "odom_est_distance:  " << odom_est_distance << endl;
  cout << "error:              " << odom_est_distance << endl;
  cout << "p :                 " << probabi1lity << endl;*/
  // (1) Future work use a more sophisticated model with coupled rotation and translation.
  //Eliazar A. E. Eliazar and R. Parr, “Learning probabilistic motion models for
  //mobile robots,” in Proc. of the International Conference on Machine
  //Learning (ICML). ACM Press, 2004, p. 32

  // (2) Future work include scan context yaw guess. If the guess is inconsistent with odometry thats an additional clue something is messed up
}


bool DebugConstraints::SearchAndAddConstraint(){

  cout << "bool DebugConstraints::SearchAndAddConstraint(){ " << endl;

  //graph_->m_graph.lock();
  ConstraintsHandler ch(graph_->GetConstraints());
  auto itr_begin = graph_->GetPoses().begin();
  auto itr_end =  graph_->GetPoses().end();

  size_t count_itrs = 0;
  ros::Time t1 = ros::Time::now();
  //graph_->m_graph.unlock();

  //cout << "Training: " << ros::Time::now() - t1 << endl;


  for(auto itr = ch.begin(ConstraintType::loop_appearance) ; itr != ch.end(ConstraintType::loop_appearance) ; itr++)
  {
    Constraint3d& C(itr->second);
    const unsigned int idx_from = itr->second.id_begin;
    const unsigned int idx_to = itr->second.id_end;
    auto cld_from = (*graph_)[idx_from].cloud_nopeaks_;
    auto cld_to = (*graph_)[idx_to].cloud_nopeaks_;
    const Eigen::Affine3d Tfrom = (*graph_)[idx_from].GetPose();
    const Eigen::Affine3d Tto = Tfrom*PoseCeresToEig(C.t_be);

    if(par_.visualize){
      CorAlignment::AlignmentQualityPlot::PublishCloud("verify_from", cld_from, Tfrom , "from");
      CorAlignment::AlignmentQualityPlot::PublishCloud("verify_to", cld_to, Tto , "to");
    }
    char c = getchar();

    std::map<string,double> quality;
    VerifyByAlignment(idx_from, Tfrom, idx_to, Tto, quality);
    ScanLearningInterface::s_scan sfrom = {Tfrom, (*graph_)[idx_from].cloud_nopeaks_, (*graph_)[idx_from].cloud_peaks_, (*graph_)[idx_from].cloud_normal_};
    ScanLearningInterface::s_scan sto =   {Tto,        (*graph_)[idx_to].cloud_nopeaks_,   (*graph_)[idx_to].cloud_peaks_,   (*graph_)[idx_to].cloud_normal_};
    Eigen::MatrixXd X_CorAl;
    Eigen::MatrixXd X_CFEAR;
    bool valid;

    sli_.PredAlignment(sfrom, sto, quality, X_CorAl, X_CFEAR, valid);
    cout << quality["coral"] << " coral: " << X_CorAl << endl;
    cout << quality["CFEAR"] << " CFEAR: " << X_CFEAR << endl;
    cout << quality["alignment_quality"] << " combined: " << X_CFEAR << endl;
    cout << sfrom.CFEAR->GetSize() << endl;
    cout << sto.CFEAR->GetSize() << endl;


    if(VerifyLoopCandidate(C)){

    }

  }
  return true;
}


template<typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1)
  {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
  {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end); // I want to ensure that start and end
  // are exactly the same as the input
  return linspaced;
}

}
