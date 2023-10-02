#include "alignment_checker/scancomparsion.h"


namespace alignment_checker{
double ScanComparsion::tolerance = 0.1;
bool ScanComparsion::verbose = false;
double ScanComparsion::GetAlignmentQuality(bool &aligned){
  aligned = aligned_;
  return merged_ - separate_;
}
double ScanComparsion::GetAlignmentQualityExtended(bool &aligned, double &merged, double &separate){
  merged = merged_;
  separate = separate_;
  return GetAlignmentQuality(aligned);
}
ScanComparsion::ScanComparsion(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, double radius, bool downsample, m_type type, double reject_ratio) :
  src_overlap_(new pcl::PointCloud<pcl::PointXYZ>),
  tar_overlap_(new pcl::PointCloud<pcl::PointXYZ>),
  src_(new pcl::PointCloud<pcl::PointXYZ>),
  target_(new pcl::PointCloud<pcl::PointXYZ>)
{
  type_ = type;
  reject_ratio_ = reject_ratio;
  if(type==determinant)
    det_only_ = true;
  else
    det_only_ = false;
  radius_ = radius;
  downsample_ = downsample;
  SetInput(src, target);
  // cout<<"center: "<<center_<<endl;
  CalculateOverlap();
  aligned_ = CheckAlignment();


}
void ScanComparsion::SetInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target){
  center_ = src->sensor_origin_+(src->sensor_origin_-target->sensor_origin_)/2;
  src->sensor_origin_ = center_;
  target->sensor_origin_ = center_;
  if(downsample_){
    pcl::VoxelGrid<pcl::PointXYZ>  sors;
    sors.setInputCloud(src);
    sors.setLeafSize(filter_distance, filter_distance, filter_distance);
    sors.filter(*src_);

    pcl::VoxelGrid<pcl::PointXYZ>  sort;
    sort.setInputCloud(target);
    sort.setLeafSize(filter_distance, filter_distance, filter_distance);
    sort.filter(*target_);
  }
  else{
    *src_ = *src;
    *target_ = *target;
  }

  ScanSrc_ = boost::shared_ptr<ScanType>(new ScanType(src_,type_));
  ScanTarget_ = boost::shared_ptr<ScanType>(new ScanType(target_,type_));

}


bool ScanComparsion::CheckAlignment(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_overlap (new pcl::PointCloud<pcl::PointXYZ>);

  (*merged_overlap)+=(*src_overlap_);
  (*merged_overlap)+=(*tar_overlap_);
  merged_overlap->sensor_origin_ = center_;

  st_src_overlap = boost::shared_ptr<ScanType>(new ScanType(src_overlap_, type_));
  st_tar_overlap = boost::shared_ptr<ScanType>(new ScanType(tar_overlap_, type_));
  st_merged = boost::shared_ptr<ScanType>(new ScanType(merged_overlap, type_));
  st_src_overlap->ComputeInformation(radius_, det_only_);
  st_tar_overlap->ComputeInformation(radius_, det_only_);
  st_merged->ComputeInformation(radius_, det_only_);

  // Remove points with lowest information from src and tar - these might have a negative impact on the final distribution;

  std::vector<double> f_ent_sep = st_src_overlap->GetEntropy(true);
  std::vector<double> f_ent_tar = st_tar_overlap->GetEntropy(true);
  f_ent_sep .insert(f_ent_sep .end(), f_ent_tar.begin(), f_ent_tar.end());
  std::sort(f_ent_sep.begin(), f_ent_sep.end());
  int i = reject_ratio_*f_ent_sep.size();

  if(i<f_ent_sep.size()){
    double th = f_ent_sep[i];
    st_tar_overlap->FilterMinEntropyLimit(th);
    st_src_overlap->FilterMinEntropyLimit(th);
    //cout<<"Rejection ratio: "<<reject_ratio_<<" filter totally: "<<i<<" points under th: "<<th<<endl;
  }


  // find consistent points between separate and merged entropy
  std::vector<bool>  consistent_idx_src = st_src_overlap->GetValidIndices();
  std::vector<bool>  consistent_idx_tar = st_tar_overlap->GetValidIndices();
  std::vector<bool>  consistent_idx_merged = st_merged->GetValidIndices();

  if ( consistent_idx_src.size()+consistent_idx_tar.size() != consistent_idx_merged.size() ) {
    cerr<<"vector size error"<<endl;
    exit(0);
  }
  for (int i=0;i<consistent_idx_merged.size();i++){
    if( i < consistent_idx_src.size())
      consistent_idx_src[i] = consistent_idx_merged[i] = consistent_idx_src[i] && consistent_idx_merged[i]; // if both are valid
    else
      consistent_idx_tar[i-consistent_idx_src.size()] = consistent_idx_merged[i] = consistent_idx_tar[i-consistent_idx_src.size()] && consistent_idx_merged[i]; // if both are valid
  }


  st_src_overlap->FilterEntropyByIndex(consistent_idx_src);
  st_tar_overlap->FilterEntropyByIndex(consistent_idx_tar);
  st_merged->FilterEntropyByIndex(consistent_idx_merged);
  std::vector<double> ent_src, ent_tar, ent_merged;

  ent_src = st_src_overlap->GetEntropy(true);
  ent_tar = st_tar_overlap->GetEntropy(true);
  ent_src.insert(ent_src.end(), ent_tar.begin(), ent_tar.end());
  size_t count = 0;
  if(type_==entropy_median)
    separate_ = ScanType::median(ent_src);
  else
    separate_ = ScanType::mean(ent_src, count);

  if(type_==mme)
    separate_ = 0;
  //cout<<"type: "<<mtype2string(type_)<<endl;

  /*double inf_src = st_src_overlap->GetInformation();
  int inf_src_size = st_src_overlap->GetInformationPointsetSize();

  double inf_tar = st_tar_overlap->GetInformation();
  int inf_tar_size = st_tar_overlap->GetInformationPointsetSize();
  separate_ = (inf_src+inf_tar)/(inf_src_size+inf_tar_size);


  */


  //double tot_merged = st_merged->GetInformation();
  //int merged_size = st_merged->GetInformationPointsetSize();
  //

  merged_ = st_merged->GetInformation()/st_merged->GetInformationPointsetSize();
  double overlap_size = src_overlap_->points.size()+tar_overlap_->points.size();
  double original_size = src_->points.size()+target_->points.size();
  if(overlap_size/original_size<min_overlap_){
    merged_ = 10;
    separate_ = 0;
  }

  //cout<<"entropy: "<<merged_<<endl;
  //cout<<"src_: "<<src_overlap_->size()<<", tar: "<<tar_overlap_->size()<<", mer: "<<merged_overlap->size()<<endl;
  if(verbose){
    cout<<"separate: "<<separate_<<endl;
    cout<<"merged: "<<merged_<<endl;
    cout<<"diff: "<<merged_ - separate_<<", r="<<radius_<<endl;
  }
  return true; //to be implemented
  //cout<<"percent: "<<(inf_merged-separate)<<endl;
}
void ScanComparsion::CalculateOverlap(){
  src_overlap_->clear();
  tar_overlap_->clear();
  ScanTarget_->GetOverlap(src_, src_overlap_, tar_overlap_, radius_);


  src_overlap_->sensor_origin_ = center_;
  tar_overlap_->sensor_origin_ = center_;

}

void ScanComparsion::StoreComparsionData(const std::string &dir, const std::string& suffix){


  std::ofstream tars, srcs, mergs;
  tars.open  (dir+"/"+"comp_tar_"+suffix);
  srcs.open  (dir+"/"+"comp_src_"+suffix);
  mergs.open (dir+"/"+"comp_merged_"+suffix);

  cout<<"save: "<<dir+"/"+"comp_tar_"+suffix<<endl;
  if(st_src_overlap==NULL || st_tar_overlap==NULL || st_merged==NULL )
    return;

  std::vector<double> veksrc = st_src_overlap->GetEntropy(true);
  std::vector<double> vektar = st_tar_overlap->GetEntropy(true);
  std::vector<double> vekmerged = st_merged->GetEntropy(true);
  cout<<"veksrc: "<<veksrc.size()<<", vektar"<<vektar.size()<<", vekmer"<<vekmerged.size()<<endl;

  for(auto &e:veksrc)
    srcs<<e<<", ";
  for(auto &e:vektar)
    tars<<e<<", ";
  for(auto &e:vekmerged)
    mergs<<e<<", ";


  cout<<"saved: "<<dir+"/"+"comp_tar_"+suffix<<endl;

  srcs.close();
  tars.close();
  mergs.close();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ScanComparsion::GetMergedDifferential(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr diff(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged = st_merged->GetScan();
  if(merged==NULL){
    cerr<<"differential information failed"<<endl;
    return NULL;
  }
  //diff->resize(merged->size());

  std::vector<double> separate_ent = st_src_overlap->GetEntropy();
  std::vector<double> tar_ent = st_tar_overlap->GetEntropy();
  separate_ent.insert( separate_ent.end(), tar_ent.begin(), tar_ent.end() );
  std::vector<double> merged_ent = st_merged->GetEntropy();

  std::vector<bool>   st_src_valid = st_src_overlap->GetValidIndices();
  std::vector<bool>   st_tar_valid = st_tar_overlap->GetValidIndices();
  st_src_valid.insert( st_src_valid.end(), st_tar_valid.begin(), st_tar_valid.end() );
  std::vector<bool>   merged_valid = st_merged->GetValidIndices();




  if(separate_ent.size()!=merged_ent.size()){
    cerr<<"Wrong size"<<endl;
    return NULL;
  }

  for(int i=0;i<merged->size();i++){
    pcl::PointXYZI p;
    p.x = merged->points[i].x;
    p.y = merged->points[i].y;
    p.z = merged->points[i].z;
    if(st_src_valid[i]!=merged_valid[i])
      cerr<<"Why are these not equal???"<<endl;

    if(st_src_valid[i] && st_src_valid[i]==merged_valid[i]){
      double e_diff = merged_ent[i]-separate_ent[i];
      p.intensity = e_diff;
      diff->push_back(p);
    }
  }


  return diff;
}
/*void ScanComparsion::SetNext( const pcl::PointCloud<pcl::PointXYZ>::Ptr &src){
  target_ = src_;
  ScanTarget_ = ScanSrc_;
  src_ = src;
  ScanSrc_ = boost::shared_ptr<ScanType>(new ScanType(src_));
  CalculateOverlap();
}*/



VisComparsion::VisComparsion(const std::string frameid): nh_("~")
{
  pub_src = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/src", 10);
  pub_tar = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/target", 10);
  pub_overlap_src = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/overlap_src", 10);
  pub_overlap_tar = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/overlap_target", 10);
  pub_overlap_merged = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/overlap_merged", 10);
  pub_diff_merged = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("/differential_merged", 10);

  frameid_ = frameid;
}

void VisComparsion::PlotClouds(boost::shared_ptr<ScanComparsion> &comp){
  ros::Time tnow = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_info;

  cloud = Stamp(comp->GetSrc(),tnow);
  if(cloud !=NULL)
    pub_src.publish(cloud);

  cloud = Stamp(comp->GetTar(),tnow);
  if(cloud !=NULL)
    pub_tar.publish(cloud);

  cloud_info = Stamp(comp->GetSrcOverlap(),tnow);
  if(cloud_info !=NULL)
    pub_overlap_src.publish(cloud_info);

  cloud_info = Stamp(comp->GetTarOverlap(),tnow);
  pub_overlap_tar.publish(cloud_info);

  cloud_info = Stamp(comp->GetMergedOverlap(),tnow);
  pub_overlap_merged.publish(cloud_info);


  cloud_info = Stamp(comp->GetMergedDifferential(),tnow);

  pub_diff_merged.publish(cloud_info);


}

void VisComparsion::PlotPoses(Eigen::Affine3d &src, Eigen::Affine3d &target){
  ros::Time tnow = ros::Time::now();
  tf::Transform src_tf;
  tf::Transform tar_tf;
  tf::poseEigenToTF(src, src_tf);
  tf::poseEigenToTF(target, tar_tf);
  tf_br.sendTransform(tf::StampedTransform(src_tf,tnow,"world","src"));
  tf_br.sendTransform(tf::StampedTransform(tar_tf,tnow,"world","target"));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisComparsion::Stamp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Time &tcloud){
  pcl_conversions::toPCL(tcloud, cloud->header.stamp);
  cloud->header.frame_id = frameid_;
  return cloud;

}
pcl::PointCloud<pcl::PointXYZI>::Ptr VisComparsion::Stamp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const ros::Time &tcloud){
  pcl_conversions::toPCL(tcloud, cloud->header.stamp);
  cloud->header.frame_id = frameid_;
  return cloud;

}



}

