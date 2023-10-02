#include "place_recognition_radar/RadarScancontext.h"

namespace PlaceRecognitionRadar
{

static float deg2rad(float degrees)
{
  return degrees * M_PI / 180.0;
}

RSCManager::RSCManager(const RSCManager::Parameters& pars) : par(pars)
{

  // Set Parent class Scan Context parameters
  this->PC_NUM_RING = par.PC_NUM_RING;
  this->PC_NUM_SECTOR = par.PC_NUM_SECTORS;
  this->SEARCH_RATIO = par.SEARCH_RATIO;
  this->NUM_CANDIDATES_FROM_TREE = par.NUM_CANDIDATES_FROM_TREE;
  this->SC_DIST_THRES = par.SC_DIST_THRES;
  this->N_candidates = par.N_CANDIDATES;
  this->PC_MAX_RADIUS = par.PC_MAX_RADIUS;
  this->PC_UNIT_SECTORANGLE = 360.0 / double(par.PC_NUM_SECTORS);
  this->PC_UNIT_RINGGAP = par.PC_MAX_RADIUS / double(par.PC_NUM_RING);

  // Set interpolation type for descriptor downscaling
  if(this->par.interpolation == "nearest_neighbor")
    this->interpolation_type = cv::INTER_NEAREST;
  else if(this->par.interpolation == "bilinear")
    this->interpolation_type = cv::INTER_LINEAR;
  else if(this->par.interpolation == "bicubic")
    this->interpolation_type = cv::INTER_CUBIC;
  else if(this->par.interpolation == "area")
    this->interpolation_type = cv::INTER_AREA;
  else
  {
    std::cout << "Interpolation type is unvalid! Set intead to bicubic..." << std::endl;
    this->interpolation_type = cv::INTER_CUBIC;
  }

}

MatrixXd RSCManager::MakeRadarContext(cv::Mat& scan_img)
{

  cv::threshold(scan_img, scan_img, par.radar_threshold, 255, cv::THRESH_TOZERO); // Threshold the original radar image (before downscale)

  cv::Mat ds_scan_mat;
  cv::resize(scan_img, ds_scan_mat, cv::Size(this->PC_NUM_SECTOR, this->PC_NUM_RING), 0 ,0 ,this->interpolation_type); //  Down scaled scan image

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ds_scan_eigen; // Eigen matrix for downscaled sc

  if(this->par.normalize) // If to normalize the image (0-255)
    cv::normalize(ds_scan_mat, ds_scan_mat, 0, 255, cv::NORM_MINMAX);
  cv::cv2eigen(ds_scan_mat, ds_scan_eigen); // Load down scaled sc from mat to eigen

  return ds_scan_eigen;
} // RSCManager::makeScancontext

MatrixXd RSCManager::MakeRadarCloudContext(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{

  TicToc t_making_desc;

  int num_pts_scan_down = cloud->points.size();

  // main
  const int NO_POINT = -1000;
  MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
  MatrixXd bin_count = MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
  std::unordered_map<std::string,std::vector<SCPointType>> pnt_map;

  SCPointType pt;
  float azim_angle, azim_range; // wihtin 2d plane
  int ring_idx, sctor_idx;
  for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
  {
    pt.x = cloud->points[pt_idx].x;
    pt.y = cloud->points[pt_idx].y;
    pt.intensity = cloud->points[pt_idx].intensity;

    // xyz to ring, sector
    azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
    azim_angle = xy2theta(pt.x, pt.y);

    // if range is out of roi, pass
    if( azim_range > PC_MAX_RADIUS )
      continue;

    ring_idx = std::max( std::min( PC_NUM_RING, int(ceil( (azim_range / PC_MAX_RADIUS) * PC_NUM_RING )) ), 1 );
    sctor_idx = std::max( std::min( PC_NUM_SECTOR, int(ceil( (azim_angle / 360.0) * PC_NUM_SECTOR )) ), 1 );
    //auto key = std::to_string(ring_idx-1) + "_" + std::to_string(sctor_idx-1);
    //pnt_map[key].push_back(pt);

    if(desc(ring_idx-1, sctor_idx-1) == NO_POINT)
      desc(ring_idx-1, sctor_idx-1) = pt.intensity;
    else{
      if(par.desc_function == "sum")
          desc(ring_idx-1, sctor_idx-1) += pt.intensity;
      else if(par.desc_function == "max")
          desc(ring_idx-1, sctor_idx-1) = std::max(desc(ring_idx-1, sctor_idx-1), (double)pt.intensity);
      else{
        cout << "Not a valid descriptor function!" << endl;
        exit(0);
      }
    }

    bin_count(ring_idx-1, sctor_idx-1)++;
  }
  double max_bin = 1;
  double max_intensity = 1;

  // Divison before no_point check, thus no_point is not set
  desc = desc / par.desc_divider;

  // reset no points to zero (for cosine dist later), and find max_bind value
  for ( int row_idx = 0; row_idx < desc.rows(); row_idx++ ){
    for ( int col_idx = 0; col_idx < desc.cols(); col_idx++ ){
      if( desc(row_idx, col_idx) == NO_POINT )
        desc(row_idx, col_idx) = par.no_point;
      if( max_bin < bin_count(row_idx, col_idx) )
        max_bin = bin_count(row_idx, col_idx);
      if( max_intensity < desc(row_idx, col_idx) )
        max_intensity = desc(row_idx, col_idx);
    }
  }
  // desc = desc.cwiseQuotient(bin_count);

  t_making_desc.toc("PolarContext making");

  return desc;
}

void RSCManager::makeAndSaveScancontextAndKeys(Eigen::MatrixXd& sc, const Eigen::Affine3d& Todom)
{
  Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
  Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
  std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

  polarcontexts_.push_back(sc);
  polarcontext_invkeys_.push_back(ringkey);
  polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);

  current_and_augments_ = { {polarcontexts_.back(), polarcontext_invkeys_.back(), polarcontext_invkeys_mat_.back(), Eigen::Affine3d::Identity()} };

  ExcludeAndUpdateLikelihood(Todom);
}

void RSCManager::makeAndSaveScancontextAndKeysRadarRaw(cv::Mat& scan_mat, const Eigen::Affine3d& Todom){
  TicToc t_making_desc;

  Eigen::MatrixXd sc = MakeRadarContext(scan_mat); // Changed paramter to scan_mat compared to original
  t_making_desc.toc("PolarContext making");
  makeAndSaveScancontextAndKeys(sc, Todom);
}

void RSCManager::makeAndSaveScancontextAndKeysRadarCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Eigen::Affine3d& Todom, bool augment)
{
  Eigen::MatrixXd sc = MakeRadarCloudContext(cloud);
  makeAndSaveScancontextAndKeys(sc, Todom); // Create and Save scan context for non-agumented scan
  //VisualizeScanContext(sc, "local map");

  if(par.augment_sc)
  {
    int count = 1;
    auto augments_vek = std::vector<std::vector<double>>( { {0.0, -2.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, -4.0, 0.0}, {0.0, 4.0, 0.0} } ); // Augmentations
    for(auto && augment : augments_vek)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr augmented(new pcl::PointCloud<pcl::PointXYZI>());
      Eigen::Affine3d Tperturbation(VectorToAffine3dxyez(augment));
      pcl::transformPointCloud( *cloud, *augmented,  Tperturbation);
      Eigen::MatrixXd sc = MakeRadarCloudContext(augmented);
      Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
      Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
      std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);
      current_and_augments_.push_back( {sc, ringkey, polarcontext_invkey_vec, Tperturbation} );
      /*ros::Time t = ros::Time::now();
      pcl_conversions::toPCL(t, augmented->header.stamp);
      CorAlignment::AlignmentQualityPlot::PublishCloud("/augment" + std::to_string(count++), augmented, Todom, "context");*/
    }
  }
  //char c = getchar();
}
void RSCManager::ExcludeAndUpdateLikelihood(const Eigen::Affine3d& Todom){

  odom_poses_.push_back(Todom); //Save current pose

  if(odom_poses_.size() <=2){ //Determine recent scans to exclude;
    NUM_EXCLUDE_RECENT = 2;
  }
  else{
    double distance = 0.0;
    NUM_EXCLUDE_RECENT = 0;
    Eigen::Affine3d Tprev = odom_poses_.back();
    for(int i = odom_poses_.size()-1 ; i >= 0 && distance < DISTANCE_EXCLUDE_RECENT ; i--){
      //std::cout<<"idx: "<< i << ", dist: " << dist << ", tot: "<< distance << std::endl;
      distance = distance + (Tprev.inverse()*odom_poses_[i]).translation().norm();;
      Tprev = odom_poses_[i];
      NUM_EXCLUDE_RECENT++;
    }
  }
  //Calculate odometry similarity scores
  Eigen::Vector3d tprev = Todom.translation();
  double odom_trav_distance = 0;

  const int idx_current = odom_poses_.size() -1;
  odom_similarity.clear();
  odom_similarity.resize(idx_current);
  for(int i = idx_current -1  ; i >= 0 ; i--){
    const Eigen::Vector3d t_i = odom_poses_[i].translation();
    odom_trav_distance += (tprev - t_i).norm();
    tprev = t_i;
    const double odom_est_distance = (Todom.translation() - t_i).norm(); // distance between poses... according to odometry at least
    const double error = std::max(odom_est_distance - 5.0 ,0.0); // if within 5 meters, always considered scans "nearby"
    const double rel_error = error/odom_trav_distance;
    const double probability = exp(-rel_error*rel_error/(2*par.odom_sigma_error*par.odom_sigma_error)); // helps reject VERY unlikely candidates
    const double similarity = 1.0 - probability;
    //if(error < 30 && odom_trav_distance > 30)
    //cout << "d: " << odom_est_distance << ", e: " << error << ", rel: " << rel_error << ", p: " << probability << endl;
    odom_similarity[i] = similarity;
  }
}


void RSCManager::VanillaKDNNSearch(std::vector<size_t>& candidate_indexes, const std::vector<float>& curr_key){
  TicToc t_tree_construction;

  if( tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
  {
    polarcontext_invkeys_to_search_.clear();
    polarcontext_invkeys_to_search_.assign( polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT ) ;


    polarcontext_tree_.reset();
    polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING/*+1*/ /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */ ); // this is actually stupid now
    // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
    t_tree_construction.toc("Tree construction");
  }
  tree_making_period_conter = tree_making_period_conter + 1;
  // knn search
  std::vector<size_t> cand_idx( NUM_CANDIDATES_FROM_TREE );
  std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

  TicToc t_tree_search;
  nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
  knnsearch_result.init( &cand_idx[0], &out_dists_sqr[0] );
  polarcontext_tree_->index->findNeighbors( knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10) );
  t_tree_search.toc("Tree search");
  candidate_indexes.insert(candidate_indexes.end(), cand_idx.begin(), cand_idx.end());
}

inline double  RSCManager::L2norm(const std::vector<float>& v1, const std::vector<float>& v2){
  float l2 = 0;
  for(int i = 0 ; i < v1.size() ; i++){
    const double err = (v1[i]-v2[i]);
    l2 += err*err;
  }
  return l2;
}
void RSCManager::OdometryNNSearch(std::vector<size_t>& candidate_indexes, const std::vector<float>& current_key){

  TicToc t_tree_construction;
  std::vector<float> curr_key = current_key;
  curr_key.push_back(0.0);
  const int idx_current = polarcontext_invkeys_mat_.size() - 1;
  KeyMat polarcontext_invkeys_to_search;
  std::vector<size_t> search_to_candidate;
  for(int idx = 0 ; idx < idx_current -1 - NUM_EXCLUDE_RECENT ; idx++){
    const double sim = odom_similarity[idx];
    //if(sim > 0.1){ // reduce into subset
    polarcontext_invkeys_to_search.push_back(polarcontext_invkeys_mat_[idx]);
    polarcontext_invkeys_to_search.back().push_back(10*sim);
    search_to_candidate.push_back(idx); //Save for conversion from subset-inddx to full-set-index
    //}
  }
  std::vector<std::pair<double,int>> dist_idx_vek;
  for(int i = 0 ; i < polarcontext_invkeys_to_search.size() ; i++){ // compute l2 norm and insert sorted
    auto cand = std::make_pair(L2norm(curr_key,polarcontext_invkeys_to_search[i]), (int)search_to_candidate[i]);
    auto it = std::lower_bound(dist_idx_vek.cbegin(), dist_idx_vek.cend(), cand); //1
    dist_idx_vek.insert(it, cand);
  }
  for(int i = 0 ; i < dist_idx_vek.size() && i < NUM_CANDIDATES_FROM_TREE ; i++ ){
    candidate_indexes.push_back(dist_idx_vek[i].second);
  }
}

std::vector<candidate> RSCManager::detectLoopClosureID(double& min_dist_ref)
{
  if( polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
  {
    return std::vector<candidate>(); // Early return
  }

  std::vector<candidate> similar_candidates;
  for(auto &&  query : current_and_augments_){ // search over all augmentations
    std::vector<size_t> candidate_indices;
    if(par.odometry_coupled_closure)
      OdometryNNSearch(candidate_indices,  query.ring_key_std);
    else
      VanillaKDNNSearch(candidate_indices, query.ring_key_std);

    for ( size_t candidate_iter_idx = 0; candidate_iter_idx < candidate_indices.size() ; candidate_iter_idx++ ) // Search over all similar (ring-key l2 norm) loop candidates
    {
      const int cand_idx = candidate_indices[candidate_iter_idx];
      MatrixXd polarcontext_candidate = polarcontexts_[cand_idx];
      std::pair<double, int> sc_dist_result = distanceBtnScanContext( query.desc, polarcontext_candidate );
      //cout << "sc:   " << sc_dist_result.first << endl;
      //cout << "odom: " << odom_similarity[candidate_idx] << endl;

      const double candidate_dist_odom = par.odometry_coupled_closure ? odom_similarity[cand_idx] : 0;
      const double candidate_dist_sc = sc_dist_result.first;
      const double candidate_dist = par.odometry_coupled_closure ? candidate_dist_sc + candidate_dist_odom : candidate_dist_sc;
      const int candidate_argmin_shift = sc_dist_result.second;
      const float candidate_align = deg2rad(sc_dist_result.second * PC_UNIT_SECTORANGLE);
      similar_candidates.push_back({candidate_dist, candidate_dist_sc, candidate_dist_odom, candidate_align, cand_idx, candidate_argmin_shift, query.Toffset});
      std::sort(similar_candidates.begin(), similar_candidates.end(), CandidateDistanceCloser);

      if(similar_candidates.size() > N_candidates)
        similar_candidates.erase(similar_candidates.end() - 1);
    }
  }

  if (similar_candidates.empty()) {
    return std::vector<candidate>();
  }
  std::vector<candidate> output_candidates;
  for(auto itr = similar_candidates.begin() ; itr != similar_candidates.end() ; itr++)
  {
    //cout << "sim candidate: " << itr->min_dist << endl;
    output_candidates.push_back(*itr);

    if (this->par.desc_plot)
    {
      MatrixXd& cand_dc = polarcontexts_[ itr->nn_idx ];
      MatrixXd cand_dc_shift = circshift(cand_dc,itr->argmin_shift);
      double dist = distDirectSC(current_and_augments_.front().desc, cand_dc_shift);
      VisualizeScanContext(cand_dc_shift, "candidate");
      //VisualizeScanContext(curr_desc, "curr_desc");
      cv::waitKey(0);
      // char cc = getchar();
    }
  }
  min_dist_ref = similar_candidates.front().min_dist;
  return output_candidates;

} // RSCManager::detectLoopClosureID


/*std::string Join( const std::map<std::string,std::string>& str_map, const bool first){
  std::string str = "";
  if(str_map.empty())
    return str;
  else if(str_map.size() == 1)
    return str_map.begin()->first;
  for(auto itr = str_map.begin() ; itr != str_map.end() ; itr++){
    str +=  first ? itr->first : itr->second;
    if( std::next(itr) != str_map.end())
      str += ",";
  }
  return str;
}*/

}

