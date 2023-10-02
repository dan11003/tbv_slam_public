
#include "alignment_checker/scan.h"
namespace alignment_checker{

double ScanType::max_swell = 0;
double ScanType::max_swell_dist = 50;
std::string mtype2string(m_type &type){
  switch(type){
  case entropy:
    return "entropy";
  case entropy_median:
    return "entropy-median";
  case determinant:
    return "determinant";
  case mme:
    return "mean-map-entropy";
  case rel_ndtp2d:
    return "rel-ndtp2d";
  default:
    return "ndtp2d";
  }
}

m_type string2mtype(std::string &type){
  if(type=="entropy")
    return entropy;
  else if(type=="entropy-median")
    return entropy_median;
  else if(type == "determinant")
    return determinant;
  else if(type == "mean-map-entropy")
    return mme;
  else if(type=="ndtp2d")
    return ndtp2d;
  else if(type=="rel-ndtp2d")
    return rel_ndtp2d;
  else{
    cerr<<"could not convert: "<<type<<endl;
    exit(0);
  }
}
ScanType::ScanType(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, m_type type):cloud_with_information_(new pcl::PointCloud<pcl::PointXYZI> ) {
  if(input==NULL){
    cerr<<"NULL Input"<<endl;
    exit(0);
  }
  cloud_ = input;
  kdtree_.setInputCloud(cloud_);
  type_ = type;
}

void ScanType::SetInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input){
  cloud_ = input;
  kdtree_.setInputCloud(cloud_);
  entropy_.clear();
  valid_pnt_.clear();

}

void ScanType::GetOverlap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_target, double radius){
  std::vector<int> idx_input;
  std::vector<int> idx_this;
  GetNeighboors(input, radius, idx_input, idx_this );
  ScanType::ExtractIndecies(input, idx_input, overlap_input);
  ScanType::ExtractIndecies(cloud_, idx_this, overlap_target);
}
void ScanType::FilterMinEntropyLimit(const double min_entropy_th){
  for(int i=0;i<cloud_->points.size();i++){
    if(entropy_[i]<min_entropy_th){
      entropy_[i] = default_entropy_;
      valid_pnt_[i] = false;
    }
  }
  Update();
}
double ScanType::median(std::vector<double> &scores)
{
  size_t size = scores.size();

  if (size == 0)
  {
    return 0;  // Undefined, really.
  }
  else
  {
    sort(scores.begin(), scores.end());
    if (size % 2 == 0)
    {
      return (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else
    {
      return scores[size / 2];
    }
  }
}
double ScanType::mean(std::vector<double> &scores, size_t &count){
  size_t size = scores.size();

  if (size == 0)
  {
    return 0;  // Undefined, really.
  }
  else
  {
    double sum = 0;
    size_t cnt = 0;
    for(auto & s : scores){
      sum += s;
      cnt++;
    }
    count = cnt;
    return sum/cnt;
  }
}
void ScanType::GetNeighboors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, double radius, std::vector<int> &idx_input, std::vector<int> &idx_this ){
  std::vector<std::vector<int> > thisIdxAgg;
  std::vector<int> inputIdxAgg;
  if(input ==NULL || input->size()==0){
    cerr<<"Input error"<<endl;
    return;
  }

  inputIdxAgg.resize(input->size(),-1);
  thisIdxAgg.resize(input->size());
  pcl::PointXYZ p_origin (this->cloud_->sensor_origin_(0), this->cloud_->sensor_origin_(1), this->cloud_->sensor_origin_(2));
#pragma omp parallel num_threads(8)
  {
#pragma omp for
    for(int i=0;i<input->size();i++){
      pcl::PointXYZ p = input->points[i];
      double d = sqrt( (p.x-p_origin.x)*(p.x-p_origin.x)+(p.y-p_origin.y)*(p.y-p_origin.y)+(p.z-p_origin.z)*(p.z-p_origin.z) );
      double r_inc = d/max_swell_dist*(ScanType::max_swell- radius);
      if( r_inc > max_swell*radius || r_inc < 0 )
        r_inc = max_swell*radius;
      double r = radius + r_inc;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointdistRadiusSearch;
      if ( kdtree_.radiusSearch (p, r, pointIdxRadiusSearch,pointdistRadiusSearch) > 3 ){
        inputIdxAgg[i] = i;
        thisIdxAgg[i]=pointIdxRadiusSearch;
      }
    }
  }


  for(int i=0;i<inputIdxAgg.size();i++){
    if(inputIdxAgg[i]!=-1)
      idx_input.push_back(inputIdxAgg[i]);
  }

  for(int i=0;i<inputIdxAgg.size();i++){
    for(int j=0;j< thisIdxAgg[i].size();j++){
      idx_this.push_back(thisIdxAgg[i][j]);
    }
  }


std:sort( idx_this.begin(), idx_this.end() );
  idx_this.erase( unique( idx_this.begin(), idx_this.end() ), idx_this.end() );


}
pcl::PointCloud<pcl::PointXYZ>::Ptr ScanType::GetLocalCloud(pcl::PointXYZ p, double radius){

  pcl::PointCloud<pcl::PointXYZ>::Ptr lcloud(new pcl::PointCloud<pcl::PointXYZ>());

  std::vector<float> pointRadiusSquaredDistance;
  std::vector<int> pointIdxRadiusSearch;
  if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
    for (int i=0;i<pointIdxRadiusSearch.size();i++) {
      lcloud->push_back(cloud_->points[pointIdxRadiusSearch[i]]);
    }
  }
  return lcloud;
}
double ScanType::FilterEntropyByIndex(std::vector<bool>& indicies){
  double sum = 0;
  int count = 0;
  if(indicies.size()!=valid_pnt_.size()){
    cerr<<"index invalid in consistency filtering"<<endl;
    exit(0);
  }
  int iconcnt = 0;
  for(int i=0;i<valid_pnt_.size();i++){
    if(valid_pnt_[i] && (!indicies[i])){
      iconcnt++;
      valid_pnt_[i] = indicies[i];
    }
  }
  //cout<<"removed: "<<iconcnt<<" inconsistencies, now "<<GetEntropy(true).size()<<endl;
  Update();

}
void ScanType::Update(){
  std::vector<double> ent = GetEntropy(true);
  if( type_ == entropy_median){
    tot_entropy_ = ScanType::median(ent);
    entropy_pointset_size_ = 1;
  }
  else
    tot_entropy_ = ScanType::mean(ent, entropy_pointset_size_ )*entropy_pointset_size_;
}

void ScanType::ComputeInformation(const double radius, bool det_only, double d_factor){
  if(cloud_->size()==0)
    return;
  if(entropy_.size()==0){
    entropy_.resize(cloud_->size(), default_entropy_);
    valid_pnt_.resize(cloud_->size(), false);
    pcl::PointXYZ p_origin (this->cloud_->sensor_origin_(0), this->cloud_->sensor_origin_(1), this->cloud_->sensor_origin_(2));
    //cout<<"origin: "<<p_origin<<endl;
#pragma omp parallel num_threads(8)
    {
#pragma omp for
      for(int i=0;i<cloud_->size();i++){
        pcl::PointXYZ p = cloud_->points[i];
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<int> pointIdxRadiusSearch;

        double d = sqrt( (p.x-p_origin.x)*(p.x-p_origin.x)+(p.y-p_origin.y)*(p.y-p_origin.y)+(p.z-p_origin.z)*(p.z-p_origin.z) );
        double r_inc = d/max_swell_dist*(ScanType::max_swell - radius);
        if( r_inc > max_swell*radius || r_inc < 0 )
          r_inc = max_swell*radius;
        double r = radius + r_inc;

        /*if(i%100==0){
          cout<<"d="<<d<<", gives r="<<r<<", max_swell_dist="<<max_swell_dist<<", max_swell="<<max_swell<<endl;
        }*/

        if ( kdtree_.radiusSearch (p, r, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 6 ){
          pcl::PointCloud<pcl::PointXYZ> Plocal;
          for(auto i:pointIdxRadiusSearch)
            Plocal.push_back(cloud_->points[i]);

          Eigen::Vector4d xyz_centroid;
          Eigen::Matrix3d C;
          pcl::computeMeanAndCovarianceMatrix(Plocal, C, xyz_centroid);
          int n = Plocal.size();
          C = n/(n-1)*C;
          double det = C.determinant();//exp(1.0)*det
          if(isnanl(det))
            continue;

          double hq;
          if(det_only)
            hq = det;
          else
            hq = 1/2.0*log(2*M_PI*exp(1.0)*det);

          if(!isnanl(hq) && !isinfl(hq)){
            entropy_[i] = hq;
            valid_pnt_[i] = true;
          }
          else{
            valid_pnt_[i] = false;
            entropy_[i] = default_entropy_;
          }
        }
      }
    }

  }
  Update();
}

void ScanType::ExtractIndecies(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, std::vector<int> &indecies, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered){
  if(input ==NULL || input->size()<indecies.size() || filtered==NULL ){
    std::cerr<<"Error Extract indecies"<<std::endl;
    return;
  }
  filtered->resize(indecies.size());
  for(int i=0;i<indecies.size();i++){
    if(indecies[i]<input->size())
      filtered->points[i] = input->points[indecies[i]];
    else
      std::cerr<<"indecies out of range"<<std::endl;
  }

}

std::vector<double> ScanType::GetEntropy(bool valid_only ){

  if(valid_only){
    std::vector<double> ent;
    for(int i = 0 ; i<entropy_.size() ; i++){
      if(valid_pnt_[i])
        ent.push_back(entropy_[i]);
    }
    return ent;
  }
  else
    return entropy_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr ScanType::GetScanWithInformation(){

  if(cloud_with_information_==NULL){
    cerr<<"NULL cloud"<<endl;
    exit(0);
  }
  cloud_with_information_->resize(cloud_->size());

  /*#pragma omp parallel num_threads(8)
  {
#pragma omp for*/
  for(int i=0;i<cloud_->size();i++){
    pcl::PointXYZI p;
    p.x = cloud_->points[i].x;
    p.y = cloud_->points[i].y;
    p.z = cloud_->points[i].z;
    if(valid_pnt_[i])
      p.intensity = entropy_[i];//255*1/(1+exp(-entropy_[i]));
    else
      p.intensity = -default_entropy_;//255*1/(1+exp(-entropy_[i]));
    cloud_with_information_->points[i] = p;
  }
  //}
  return cloud_with_information_;
}




}
