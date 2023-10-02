#include "tbv_slam/posegraph.h"
namespace tbv_slam{



PoseGraph::PoseGraph(PoseGraph::Parameters& par) :  par_(par), nh_("~"), eval_manager(par_.eval_manager_par){
  if(par_.online) {
    thread_vec.push_back(new std::thread(&PoseGraph::AddNodeThread, this));
    thread_vec.push_back(new std::thread(&PoseGraph::OptimizerThread, this));
  }
  thread_vec.push_back(new std::thread(&PoseGraph::AddConstraintThread, this));

}


/*void PoseGraph::AddNodeThSafe(const RadarScan& scan, const std::vector<Constraint3d>& constraints){
  queue_new_nodes_.enqueue(std::make_pair(scan,constraints));
}*/
void PoseGraph::AddNodeThSafe(std::pair<RadarScan, std::vector<Constraint3d>>& node_constraint) {
  queue_new_nodes_.enqueue(node_constraint);
}

void PoseGraph::AddSimpleGraphUnsafe(simple_graph& sg){ //Thread safe
  m_graph.lock();
  auto itr = sg.begin();
  while(itr != sg.end()){
    nodes_.Add(itr->first);
    for(auto && c : itr->second)
      constraints_.Add(c);
    itr = sg.erase(itr);
  }
  m_graph.unlock();
}
void PoseGraph::AddConstraintThSafe(const Constraint3d& constraint){
  queue_new_loopconstraints_.enqueue(constraint);
}


void PoseGraph::AddConstraintThread(){
  ros::Rate r(20);
  bool node_inserted = true;
  while (keep_running && ros::ok()) {
    r.sleep();
    while(!queue_new_loopconstraints_.empty()){
      auto constraint = queue_new_loopconstraints_.dequeue(); // wait
      m_graph.lock();
      constraints_.Add(constraint);
      m_graph.unlock();
    }
  }
}
void PoseGraph::AddNodeThread(){
  ros::Rate r(20);
  bool node_inserted = true;
  while (keep_running && ros::ok()) {
    r.sleep();

    while(!queue_new_nodes_.empty()){
      qnode node_and_constraints = queue_new_nodes_.dequeue(); // wait
      m_graph.lock();
      if (node_and_constraints.second.size() > 0) {
        node_and_constraints.first.T = Pose3d(nodes_.GetScan(node_and_constraints.second.front().id_end).GetPose() * node_and_constraints.second.front().t_be.GetPose().inverse());
      }
      nodes_.Add(node_and_constraints.first);

      if(nodes_.size() >= 2)
        traveled_ +=  (std::prev(nodes_.end(),1)->second.T.p - std::prev(nodes_.end(),2)->second.T.p).norm(); // works only with ordered map
      for(auto && c : node_and_constraints.second)
        constraints_.Add(c);
      m_graph.unlock();
    }
  }
}
/*void PoseGraph::AddNode(const Eigen::Affine3d &Tdiff , pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,Eigen::Matrix<double,6,6> &Cov, int id, int id_prev){

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
  //pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::RandomSample<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setSample(10000);
  //sor.setLeafSize (0.04f, 0.04f, 0.04f);
  sor.filter (*downsampled);

  node n;
  n.cloud_downsampled_ = downsampled;
  n.cloud = cloud;
  n.id = id;

  std::cerr << "PROBLEM WITH IMPLEMENTATION, WRONG CONSTRUCTOR IS USED, no timing" << std::endl;
  Constraint3d c{id_prev, id, PoseEigToCeres(Tdiff), Cov.inverse()};
  n.constraint = c;
  traveled_ += Tdiff.translation().norm();

  m_newnodes.lock();
  new_nodes_.push_back(n);
  m_newnodes.unlock();
}*/
/*void PoseGraph::AddLoopConstraint(const Eigen::Affine3d &Tdiff, Eigen::Matrix<double,6,6> &Cov, int id_from, int id_to, bool odom_constraint){
  Constraint3d c;
  c.id_begin = id_from;
  c.id_end = id_to;
  c.information = Cov.inverse();
  c.t_be = PoseEigToCeres(Tdiff);
  if(odom_constraint)
    AddConstraint(odom_constraints_, c);
  else
    AddConstraint(loop_constraints_, c);
}*/



void PoseGraph::ForceOptimize(){
  if(!par_.disable_optimization){

    while(!InputQueueEmpty())
      usleep(100);

    m_graph.lock();

    ros::Time t0 = ros::Time::now();
    if (nodes_.size() > 1) {
      CeresLeastSquares optimizer(constraints_, nodes_, par_.opt_pars.ceres_pars);
      optimizer.Solve();
    }
    ros::Time t1 = ros::Time::now();
    CFEAR_Radarodometry::timing.Document("Pose grapgh optimization", CFEAR_Radarodometry::ToMs(t1-t0));
    //OutputGraph();
    m_graph.unlock();
  }
}

void PoseGraph::OptimizerThread(){
  double optimization_rate = 1.0/std::max(par_.Toptimize,0.001);
  cout<<"Started optimization at rate :"<<optimization_rate<<endl;
  ros::Rate r(optimization_rate);
  //int curr_surplus = all_constraints_.size()-GetPosesSize()+1; // measure
  //int prev_surplus = curr_surplus;
  while (keep_running && ros::ok()) {
    //m_graph.lock();     // ForceOptimize() locks, so this lock produces a deadlock
    //curr_surplus = all_constraints_.size()-GetPosesSize()+1; // measure
    if(!par_.disable_optimization && ( optimize /*|| (curr_surplus>0 && curr_surplus>prev_surplus)*/ )){
      ForceOptimize();
      //optimize = false;
      //prev_surplus = curr_surplus;
    }
    // m_graph.unlock();
    r.sleep();
  }
}

double PoseGraph::TraveledDistance(unsigned int id1, unsigned int id2){
  double distance = 0.0;
  double idx_min = std::min(id1,id2);
  double idx_max = std::max(id1,id2);
  auto itr_first = nodes_.Get(idx_min);
  auto itr_last = nodes_.Get(idx_max);
  auto itr_end = std::prev(nodes_.end());

  for(auto itr_curr = itr_first ;  itr_curr!= itr_last && itr_curr != itr_end ; itr_curr++ ){
    Constraint3d c;
    auto itr_next = std::next(itr_curr);
    if(constraints_.FindConstraint(itr_curr->first, itr_next->first , c))
      distance += c.t_be.p.norm();
    else{
      cerr << "no constraint between" << id1 << " and " << id2 << endl;
    }
  }
  return distance;
}
double PoseGraph::EuclidianDistance(unsigned int id1, unsigned int id2){
  return (nodes_.GetScan(id1).T.p - nodes_.GetScan(id2).T.p).norm();
}

std::string PoseGraph::ToString(){
  return "nodes: " + std::to_string(nodes_.size()) + ", constriants: " + constraints_.ToString();
}
void PoseGraph::SaveGraphString(const std::string& path){
  std::ofstream myfile;
  myfile.open (path);
  cout << "Save: " << nodes_.size() << " nodes to: " << path << endl;
  m_graph.lock();
  for(auto itr = nodes_.begin(); itr != nodes_.end(); itr++)
    myfile << itr->second.ToString() << endl;
  myfile.close();
  m_graph.unlock();

}

/*void PoseGraph::SavePointCloud(std::string& dir){
  int i=0;
  for (MapOfPoses::const_iterator itr = poses_.begin() ; itr != poses_.end(); ++itr) {
    pcl::PCDWriter wr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*clouds_[(*itr).first], *tmp, poses_[(*itr).first].p, poses_[(*itr).first].q);
    std::string path(dir+"cloud_"+std::to_string(i++)+".pcd");
    cout<<"Save : "<<path<<endl;
    wr.writeBinary(path,*tmp);
  }
}*/

void PoseGraph::OutputGraph(){
  const std::string est_path = par_.est_output_dir+"/00.txt";
  const std::string gt_path = par_.gt_output_dir+"/00.txt";
  std::ofstream estfile(est_path);
  std::ofstream gtfile(gt_path);
  //cout << "save trajectory..." << endl;
  //cout << "est: " << est_path << endl;
  //cout << "gt: " << gt_path << endl;

  estfile<< std::fixed << std::showpoint;
  gtfile<< std::fixed << std::showpoint;
  bool gt_found = false;
  //assert(m.rows()== 4 && m.cols()==4);
  for(auto itr = nodes_.begin() ; itr != nodes_.end() ; itr++ ){
    if(itr->second.has_Tgt_){
      Eigen::MatrixXd mgt(itr->second.GetPose().matrix());
      Eigen::MatrixXd mest(PoseCeresToEig(itr->second.Tgt).matrix());
      estfile << MatToString(mgt)  << std::endl;
      gtfile  << MatToString(mest) << std::endl;
      gt_found = true;
    }
  }
  // If no ground truth exist at all, save just est
  if(!gt_found){
    for(auto itr = nodes_.begin() ; itr != nodes_.end() ; itr++ ){
      Eigen::MatrixXd mest(itr->second.GetPose().matrix());
      estfile << MatToString(mest)  << std::endl;
    }
  }

  estfile.close();
  gtfile.close();

}
void PoseGraph::Align(){
  int count = 0;
  for(auto itr = nodes_.begin(); itr != nodes_.end(); itr++){
    if (itr->second.has_Tgt_)
      count ++;
  }
  Eigen::MatrixXd mest(count,3), mgt(count,3);
  size_t idx = 0;
  for(auto itr = nodes_.begin(); itr != nodes_.end(); itr++){
    if (itr->second.has_Tgt_) {
      mest.block<1,3>(idx,0) = itr->second.T.p.transpose();
      mgt.block<1,3>(idx++,0) = itr->second.Tgt.p.transpose();
    }
  }
  Eigen::Affine3d Tgtalign = best_fit_transform(mgt,mest).inverse(); // find optimal aligmment
  cout << "optimal: " <<Tgtalign.matrix()<<endl;
  for(auto&& itr = nodes_.begin(); itr != nodes_.end(); itr++){
      itr->second.T = PoseEigToCeres(Tgtalign*itr->second.GetPose());
  }
  int N = 0;
  double d = 0.0;
  for(auto&& itr = nodes_.begin(); itr != nodes_.end(); itr++){
    if(itr->second.has_Tgt_){
      d += (itr->second.Tgt.p - itr->second.GetPose().translation()).norm();
      N++;
    }
  }
  cout << "ATE" << d/(std::max(N,1));
}
void PoseGraph::SaveGraph(const std::string& path, PoseGraphPtr &graph){
  if(graph==NULL){
    std::cout<<"Nothing to save"<<endl;
    return;
  }
  try {
    std::cout<<"Save graph to: "<<path<<endl;
    std::ofstream ofs(path);
    boost::archive::binary_oarchive oa(ofs);
    oa << graph;
    ofs.close();
  }catch (std::exception &e) {
  }
}

bool PoseGraph::LoadGraph(const std::string& path, PoseGraphPtr &graph){

  try {
    std::ifstream ifs(path);
    boost::archive::binary_iarchive ia(ifs);
    ia >> graph;
    ifs.close();
    cout << "Graph succesfully loaded from: " << path << endl;
    cout << graph->ToString() << std::endl;

    return true;
  }catch (std::exception &e) {
    std::cerr<<"Graph could not be loaded from: "<<path<<std::endl;
    return false;
  }
}

bool PoseGraph::LoadSimpleGraph(const std::string& path, PoseGraphPtr &graph, PoseGraph::Parameters& par){
  simple_graph sg;
  std::vector<std::tuple<int,int>> asd;
  try{
    bool status = CFEAR_Radarodometry::LoadSimpleGraph(path, sg);
    graph = PoseGraphPtr(new PoseGraph(par));
    graph->AddSimpleGraphUnsafe(sg);

    while(!graph->InputQueueEmpty()){
      usleep(100);
    }
    cout << "Loaded: " << graph->size() << " nodes" << endl;
    return true;
  }
  catch (const std::exception& e) {
    cout << "Could not load simple graph" << endl;
    exit(0);
  }
  return false;
}


Eigen::Matrix<double, 6, 6> PoseGraph::GetCovariance(unsigned int from, unsigned int to){
  Eigen::Matrix<double, 6, 6> Cov = Eigen::MatrixXd::Zero(6,6);
  Eigen::Matrix<double, 6, 6> Inf = Eigen::MatrixXd::Zero(6,6);
  for(unsigned int i=from;i<to;i++){
    unsigned int from = i, to = i+1;
    Constraint3d c;
    bool found = constraints_.FindConstraint(from, to, c);
    Inf += c.information;
  }
  Cov = Inf.inverse();
  return Cov;
}


void PoseGraph::UpdateStatistics(const std::pair<unsigned int,unsigned int> & guess, const Eigen::Affine3d& Tguess, const std::map<std::string,double>& quality, int guess_nr){

  double nearest_loop_distance = 100000;
  double candidate_loop_distance = -1;
  const unsigned int from = guess.first;
  const unsigned int to = guess.second;
  const Eigen::Affine3d Tposefrom = PoseCeresToEig(nodes_.GetScan(from).Tgt);
  const Eigen::Affine3d Tposeto = PoseCeresToEig(nodes_.GetScan(to).Tgt);
  const Eigen::Affine3d Tgt_diff = Tposefrom.inverse()*Tposeto;
  const Eigen::Affine3d Terror = Tguess.inverse()*Tgt_diff;


  Eigen::Affine3d Tclosest = Tposefrom;
  unsigned int close = from;

  //if(guess.first == guess.second){
    //eval_manager.updateResults(Tposefrom, Tposefrom, nearest_loop_distance, 0.0, quality, from, to, 0);

  if(nodes_.GetScan(from).has_Tgt_){

    if(nodes_.GetScan(to).has_Tgt_){
      candidate_loop_distance = (Tposefrom.translation() - Tposeto.translation()).norm();
    }

    for(auto itr = nodes_.begin() ; itr != nodes_.end() && itr->second.idx_ < from; itr++){ // Search for closest pose before
      double idx_diff = fabs((double)from - (double)itr->second.idx_);
      if(itr->second.idx_ != from && idx_diff  > 10 && itr->second.has_Tgt_){ //
        const Eigen::Affine3d Tsearch = PoseCeresToEig(itr->second.Tgt);
        const double distance = (Tposefrom.translation() - Tsearch.translation()).norm();
        if(distance < nearest_loop_distance){
          nearest_loop_distance  = distance;
          Tclosest = Tsearch;
          close = std::distance(nodes_.begin(),itr);
        }
      }
    }
  }
  const bool loop_exist = nearest_loop_distance < 10;
  eval_manager.updateResults(Tposefrom, Tposeto, Tclosest, Terror, nearest_loop_distance, candidate_loop_distance, quality, from, to, close, guess_nr);
}

//**************************' PoseGraphVis *******************************/

void PoseGraphVis::Parameters::GetParametersFromRos( ros::NodeHandle& nh){
  nh.param<std::string>("map_optimized_topic", optimized_map_topic_, "map_optimized");
  nh.param<std::string>("loop_constraints_topic", constraints_loop_topic_, "loop_constraints");
  nh.param<std::string>("odom_constraints_topic", constraints_odom_topic_, "odom_constraints");
  nh.param<std::string>("constraints_appearance_topic", constraints_candidate_topic_, "candidate_constraints");

  nh.param<std::string>("world_frame", world_frame, "map");
  nh.param<int>("T_vis", Tupd, 2);
  nh.param<int>("skip_frames", skip_frames, 1);
  nh.param<bool>("disable_odom_constraints", disable_odom_constraints_, false);
  nh.param<bool>("visualize", visualization_enabled_, true);
}

PoseGraphVis::PoseGraphVis(PoseGraphPtr graph, const Parameters& par) : par_(par), nh_("~") {
  vis_clouds = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(par_.optimized_map_topic_, 10);
  vis_gt_clouds = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >(par_.optimized_map_topic_+"_gt", 10);
  pub_gt_path = nh_.advertise<nav_msgs::Path>(par_.gt_path_topic_, 10);
  pub_odom_path = nh_.advertise<nav_msgs::Path>("odom_path", 10);
  pub_slam_path = nh_.advertise<nav_msgs::Path>("slam_path", 10);
  graph_ = graph;
  if(par_.threading)
    th = new std::thread(&PoseGraphVis::VisualizeThread, this);
}


void PoseGraphVis::PlotAll(RadarScanHandler& scans, ConstraintsHandler& constraints_){



}

visualization_msgs::Marker PoseGraphVis::CreateDefaultMarker(const std::string& ns){
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.header.frame_id = par_.world_frame;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.3;
  m.scale.y = m.scale.x*2;
  m.scale.z = 0.5;
  m.color.a = 0.5;
  m.color.r = 1; m.color.g = 0; m.color.b = 0;
  m.ns = ns;
  m.lifetime = ros::Duration(0);
  m.pose.position.x = 0;
  m.pose.position.y = 0;
  m.pose.position.z = 0;
  m.pose.orientation.x = 0;
  m.pose.orientation.x = 0;
  m.pose.orientation.w = 1;
  m.pose.orientation.z = 0;
  return m;
}
visualization_msgs::MarkerArray PoseGraphVis::ConstriantsToMarker(ConstraintType ct, ConstraintsHandler& ch,  RadarScanHandler& sh, const std_msgs::ColorRGBA& color, bool absolute, bool ground_truth, const double& z_offset){

  ros::Time t = ros::Time::now();
  int id = 0;
  visualization_msgs::MarkerArray marr;

  const std::string c_name = Constraint2String(ct);
  visualization_msgs::Marker m = CreateDefaultMarker(Constraint2String(ct));


  for(auto itr = ch.begin(ct) ; itr != ch.end(ct) ; itr++){

    if(std::distance(ch.begin(ct), itr) % par_.skip_frames != 0)
      continue;
    const Constraint3d c = itr->second;

    Pose3d posefrom = sh.GetScan(itr->second.id_begin).T;
    Pose3d poseto = absolute ? sh.GetScan(itr->second.id_end).T : posefrom*c.t_be;
    if(ground_truth){
      if(sh.GetScan(itr->second.id_begin).has_Tgt_ && sh.GetScan(itr->second.id_end).has_Tgt_){
        Eigen::Affine3d diff = PoseCeresToEig(sh.GetScan(itr->second.id_begin).Tgt).inverse()*PoseCeresToEig(sh.GetScan(itr->second.id_end).Tgt);
        poseto = PoseEigToCeres(PoseCeresToEig(posefrom)*diff);
      }
      else {
        id++;
        continue;
      }
    }

    const Eigen::Vector3d vend = poseto.p;
    geometry_msgs::Point pfrom, pto;
    tf::pointEigenToMsg(posefrom.p, pfrom);
    tf::pointEigenToMsg(vend, pto);
    pfrom.z = pto.z = z_offset; //adjust height
    m.header.stamp = t;
    m.scale.z = 0.3;
    m.id = id;
    m.ns = c_name + c.info;
    m.color = color;
    m.points.clear();
    m.points.push_back(pfrom);
    m.points.push_back(pto);
    marr.markers.push_back(m);


    id++;
  }
  return marr;
}
void PoseGraphVis::PublishConstraints( ConstraintsHandler& ch,  RadarScanHandler& sh){
  std_msgs::ColorRGBA blue, red, green, black;
  blue.a =  1; blue.b =  1; blue.r =  0; blue.g =  0;
  red.a =   1; red.b =   0; red.r =   1; red.g =   0;
  green.a = 1; green.b = 0; green.r = 0; green.g = 1;
  black.a = 1; black.b = 1; black.r = 1; black.g = 1;

  visualization_msgs::MarkerArray text_odom, text_candidate, text_loop;
  visualization_msgs::MarkerArray odom_marr =  ConstriantsToMarker(ConstraintType::odometry, ch, sh, blue, false, false, 0.05);

  visualization_msgs::MarkerArray candidate_marr =  ConstriantsToMarker(ConstraintType::candidate, ch, sh, green, true, false, -0.3);

  visualization_msgs::MarkerArray loop_marr =  ConstriantsToMarker(ConstraintType::loop_appearance, ch, sh, green, false, false, 0.3);

  visualization_msgs::MarkerArray gt_marr =  ConstriantsToMarker(ConstraintType::loop_appearance, ch, sh, red, false, true, 2);

  visualization_msgs::MarkerArray text_marr;

  //This one is actually hard to visualize
  /*for(auto itr = ch.begin(ConstraintType::candidate) ; itr != ch.end(ConstraintType::candidate) ; itr++){
    size_t idx = std::distance(ch.begin(ConstraintType::candidate), itr);
    visualization_msgs::Marker m_text = loop_marr.markers[idx];

    m_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m_text.ns = "constraint_text";
    std::string podom = (itr->second.quality.find(ODOM_BOUNDS) != itr->second.quality.end()) ? std::to_string(itr->second.quality[ODOM_BOUNDS]) : "-";
    std::string psc = (itr->second.quality.find(SC_SIM) != itr->second.quality.end()) ? std::to_string(itr->second.quality[SC_SIM]) : "-";
    m_text.text =  "SC: " + psc + ", ODOM: " + podom ;
    m_text.color = black;
    m_text.scale.x = m_text.scale.y = m_text.scale.z = 1;
    geometry_msgs::Point pmid;
    m_text.pose.position.x = (m_text.points.front().x + m_text.points.back().x)/2.0;
    m_text.pose.position.y = (m_text.points.front().y + m_text.points.back().y)/2.0;
    m_text.pose.position.z = (m_text.points.front().z + m_text.points.back().z)/2.0;
    pmid.z = pmid.z + 2;
    text_marr.markers.push_back(m_text);
  }*/

  PublishMarkerArray(par_.constraints_odom_topic_,       odom_marr);
  PublishMarkerArray(par_.constraints_candidate_topic_,  candidate_marr);
  PublishMarkerArray(par_.constraints_loop_topic_,       loop_marr); //
  //PublishMarkerArray(par_.constraints_loop_topic_,       text_marr);
  PublishMarkerArray(par_.constraints_gt_loop_topic_,    gt_marr);

}

void PoseGraphVis::ProcessFrame(){

  if(!this->par_.visualization_enabled_)
    return;

  //cout << "visualize frame lock halt" << endl;
  graph_->m_graph.lock();
  //cout << "visualize frame lock aquired" << endl;
  cout << "visualize graph" << endl;

  std::vector<unsigned int> pose_id;
  ros::Time t =  ros::Time::now();

  //std::vector<Eigen::Affine3d> poses;
  //for(auto itr = graph_->GetPoses().begin(); itr != graph_->GetPoses().end(); itr++)
  //        poses.push_back(itr->second.GetPose());

  pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_gt(new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<tf::StampedTransform> trans_vek;


  int count = 0;
  for(auto && itr = graph_->GetPoses().begin(); itr != graph_->GetPoses().end() ; itr++)
    if (itr->second.has_Tgt_)
      count ++;
  ROS_DEBUG_DELAYED_THROTTLE(5, "void PoseGraphVis::ProcessFrame()");

  Eigen::MatrixXd mest(count,3), mgt(count,3);
  size_t idx = 0;
  for(auto itr = graph_->GetPoses().begin(); itr != graph_->GetPoses().end(); itr++){
    if (itr->second.has_Tgt_) {
      mest.block<1,3>(idx,0) = itr->second.T.p.transpose();
      mgt.block<1,3>(idx++,0) = itr->second.Tgt.p.transpose();
    }
  }
  // Eigen::Affine3d Tgtalign = Eigen::Affine3d::Identity(); // = best_fit_transform(mgt,mest); // find optimal aligmment
  // for(auto itr = graph_->GetPoses().begin(); itr != graph_->GetPoses().end(); itr++){
  //   if (itr->second.has_Tgt_){
  //     Tgtalign = itr->second.GetPose()*PoseCeresToEig(itr->second.Tgt).inverse();
  //     break;
  //   }
  // }

  Eigen::Affine3d Tgtalign = Eigen::Affine3d::Identity();

  nav_msgs::Path gt_path;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = gt_path.header.frame_id = par_.world_frame;
  pose.header.stamp = gt_path.header.stamp = t;
  for(auto itr = graph_->GetPoses().begin(); itr != graph_->GetPoses().end(); itr++){
    if(std::distance(graph_->GetPoses().begin(), itr) % par_.skip_frames != 0)
      continue;

    pcl::PointCloud<pcl::PointXYZI> tmp; // Create map
    pcl::transformPointCloud(*itr->second.cloud_peaks_, tmp, itr->second.GetPose());
    *merged += tmp;

    if(itr->second.has_Tgt_){ // Create ground truth map
      pcl::PointCloud<pcl::PointXYZI> tmp_gt;
      pcl::transformPointCloud(*itr->second.cloud_peaks_, tmp_gt, Tgtalign*PoseCeresToEig(itr->second.Tgt));
      *merged_gt += tmp_gt;
    }


    tf::Transform Tf;
    tf::transformEigenToTF(itr->second.GetPose(), Tf);
    //trans_vek.push_back(tf::StampedTransform(Tf, ros::Time::now(), par_.world_frame, std::string("pose_")+std::to_string(itr->second.idx_)));


    if(itr->second.has_Tgt_){
      auto Tgt = Tgtalign*PoseCeresToEig(itr->second.Tgt);
      tf::transformEigenToTF(Tgt, Tf);
      //trans_vek.push_back(tf::StampedTransform(Tf, ros::Time::now(), par_.world_frame, std::string("gt_")+std::to_string(itr->second.idx_)));

      tf::poseEigenToMsg(Tgt,pose.pose);
      gt_path.poses.push_back(pose);
    }
  }
  if(graph_->size()>0){
    Eigen::Affine3d T = std::prev(graph_->GetPoses().end())->second.GetPose();
    tf::Transform Tf;
    tf::transformEigenToTF(T, Tf);
    trans_vek.push_back(tf::StampedTransform(Tf, ros::Time::now(), par_.world_frame, std::string("latest")));
  }

  force_update_ = false;
  graph_->optimized = false;

  ConstraintsHandler constraints(graph_->GetConstraints());
  RadarScanHandler sh(graph_->GetPoses());
  PublishConstraints(constraints, sh);
  //cout << "Graph Info: " << graph_->ToString() << endl;
  //cout << "visualize frame releasing" << endl;
  graph_->m_graph.unlock();
  //cout << "visualize frame released" << endl;

  Tb.sendTransform(trans_vek);
  merged_gt->header.frame_id = merged->header.frame_id = par_.world_frame;
  pcl_conversions::toPCL(ros::Time::now(), merged->header.stamp);
  pcl_conversions::toPCL(ros::Time::now(), merged_gt->header.stamp);
  vis_clouds.publish(*merged);
  vis_gt_clouds.publish(*merged_gt);
  pub_gt_path.publish(gt_path);

  // Odometry/Slam trajectory path 
  nav_msgs::Path odom_path, slam_path;
  geometry_msgs::PoseStamped pose_odom, pose_slam;
  pose_odom.header.frame_id = odom_path.header.frame_id = par_.world_frame;
  pose_slam.header.frame_id = slam_path.header.frame_id = par_.world_frame;
  pose_odom.header.stamp = odom_path.header.stamp = t;
  pose_slam.header.stamp = slam_path.header.stamp = t;
  Eigen::Affine3d Todom = Eigen::Affine3d::Identity();
  Eigen::Affine3d Tslam = Eigen::Affine3d::Identity();
  for(auto itr = graph_->GetPoses().begin(); itr != graph_->GetPoses().end(); itr++){
    if(itr != graph_->GetPoses().begin()){
      Todom = Todom*constraints.RelativeMotion(std::prev(itr)->second.idx_, itr->second.idx_).inverse();
      Tslam = itr->second.GetPose();
      tf::poseEigenToMsg(Todom,pose_odom.pose);
      odom_path.poses.push_back(pose_odom);
      tf::poseEigenToMsg(Tslam,pose_slam.pose);
      slam_path.poses.push_back(pose_slam);
    }
  }
  pub_odom_path.publish(odom_path);
  pub_slam_path.publish(slam_path);
}

void PoseGraphVis::VisualizeThread(){

  ros::Time tprev = ros::Time::now();
  ros::Rate r(100);
  while (ros::ok() && !despawn_thread_) {

    if(par_.visualization_enabled_ && (ros::Time::now()-tprev > ros::Duration(par_.Tupd) || force_update_ || graph_->optimized) ){
      ProcessFrame();
      tprev = ros::Time::now();
    }
    r.sleep();
  }
}

void PoseGraphVis::pubTFForPose(const std::vector<Eigen::Affine3d>& pose, const std::vector<std::string>& name, const ros::Time& t)
{

  if (pose.size() != name.size())
    return;
  static tf::TransformBroadcaster Tbr;
  tf::Transform Tf;
  std::vector<tf::StampedTransform> trans_vek;

  for(size_t i = 0 ; i < pose.size(); i++){
    tf::transformEigenToTF(pose[i], Tf);
    trans_vek.push_back(tf::StampedTransform(Tf, t, "world", name[i]));
  }
  Tbr.sendTransform(trans_vek);
}

void PublishMarkerArray(const std::string& topic, visualization_msgs::MarkerArray& msg){
  typedef std::map<std::string,ros::Publisher> Publishers;
  static Publishers pubs;
  Publishers::iterator it = pubs.find(topic);
  if (it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] =  nh.advertise<visualization_msgs::MarkerArray>(topic,100);
    it = pubs.find(topic);
  }
  it->second.publish(msg);
}


}
