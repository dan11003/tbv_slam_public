#include "alignment_checker/alignmenttester.h"

namespace alignment_checker {

AlignmentTester::AlignmentTester(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clouds,std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > &poses,std::vector<int> &ignore, double radius, measurement_type type, bool downsample, double rejection_ratio)
{
  ignore_idx_ = ignore;
  rejection_ratio_ = rejection_ratio;
  radius_ = radius;
  clouds_ = clouds;
  poses_ = poses;
  type_ = type;

  method = mtype2string(type);


  downsample_ = downsample;
  srand((unsigned)time(NULL));
  ros::NodeHandle nh("~");
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("points11", 10);
  pub2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("points22", 10);

}
/*void AlignmentTester::PerformAndSaveTest(const std::string &dir){
  for(int i=0 ; i+1<clouds_.size() && ros::ok();i++){
    boost::shared_ptr<ScanComparsion> comp = boost::shared_ptr<ScanComparsion>(new ScanComparsion(cloud_small_terror_[i+1],clouds_[i]));
    comp->StoreComparsionData(dir,i);
  }
}*/
void AlignmentTester::PerformAndSaveTest(const std::string &dir, const std::string &filename,const std::string &dataset){
  std::ofstream ofs;
  std::string path = dir+filename+method+"_r="+std::to_string(radius_)+std::string("_d=")+std::to_string(pos_offset)+std::string("_a=")+std::to_string(angle_offset)+std::string(".txt");
  ofs.open(path);

  ofs<<"Method, Merged, Separate, Differential, Dataset, Radius, Offset_x, Offset_y, Offset_theta, Label"<<endl;
  for(int i=0 ; i+1<clouds_.size() && ros::ok();i++){
    bool ignore = false;
    for(auto j:ignore_idx_){
      if(j==i)
        ignore = true;
    }
    if(ignore){
      cout<<"Ignore: "<<i<<endl;
      continue;
    }
    boost::shared_ptr<ScanComparsion> comp_error, comp_aligned;
    //cout<<"Correct"<<endl;
    if(type_ == ndtp2d||type_==rel_ndtp2d){
      comp_error = boost::shared_ptr<ScanComparsion>(new NdtScanComparsion(clouds_error_[i+1], clouds_[i], radius_, downsample_, type_));
      comp_aligned = boost::shared_ptr<ScanComparsion>(new NdtScanComparsion(clouds_[i+1], clouds_[i], radius_, downsample_, type_));
    }
    else {
      comp_error = boost::shared_ptr<ScanComparsion>(new ScanComparsion(clouds_error_[i+1], clouds_[i], radius_, downsample_, type_, rejection_ratio_));
      comp_aligned = boost::shared_ptr<ScanComparsion>(new ScanComparsion(clouds_[i+1], clouds_[i], radius_, downsample_, type_, rejection_ratio_));
    }


    double aligned_sep, aligned_merged, error_sep, error_merged, aligned_diff, misaligned_diff;

    bool aligned;
    comp_aligned->GetAlignmentQualityExtended(aligned, aligned_merged, aligned_sep);
    comp_error->GetAlignmentQualityExtended(aligned, error_merged,error_sep);

    //comp_aligned->StoreComparsionData(dir, "aligned"+std::to_string(i)+".csv");
    //comp_error->StoreComparsionData(dir, "error"+std::to_string(i)+".csv");
    aligned_diff = aligned_merged - aligned_sep;
    misaligned_diff = error_merged - error_sep;
    cout<<"scan: "<<i<<", diff aligned: "<<aligned_diff<<", diff misaligned: "<<misaligned_diff<<"offset: "<<offsets_[i+1](0)<<", "<<offsets_[i+1](1)<<", "<<offsets_[i+1](5)<<endl;

    Eigen::Matrix<double,6,1> o = offsets_[i+1];


    ofs<<method<<", "<<std::to_string(aligned_merged)<<", "<<std::to_string(aligned_sep)<<", "<<std::to_string(aligned_diff)<<   ", "<<dataset<<", "<<radius_<<", "<<0<<   ", "<<0<<   ", "<<0<<", "<<1<<endl;
    ofs<<method<<", "<<std::to_string(error_merged)<<  ", "<<std::to_string(error_sep)<<  ", "<<std::to_string(misaligned_diff)<<", "<<dataset<<", "<<radius_<<", "<<o(0)<<", "<<o(1)<<", "<<o(5)<<", "<<0<<endl;
  }
  ofs.close();
  cout<<"Test saved at: "<<path<<endl;

}

double AlignmentTester::GausianNoiseGen(double dev){
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0,1);

  return distribution(generator)*dev;
}
void AlignmentTester::AllocateScans(){
  clouds_error_.resize(clouds_.size());
  Terror_.resize(clouds_.size());
  offsets_.resize(clouds_.size());
  for(int i=0;i<clouds_.size();i++){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudcpy(new pcl::PointCloud<pcl::PointXYZ>);
    clouds_error_[i] = cloudcpy;
  }
}
/*void AlignmentTester::ReAlignScansSmallOffset(){
  cout<<"Small error"<<endl;
  AllocateScans();
  for(int i=0;i<clouds_error_.size()&& ros::ok();i++){
    Eigen::Matrix<double,6,1> offset;
    const double sigmax=0.1,sigmay=0.1,sigmaez=0.05;
    double x=GausianNoiseGen(sigmax), y=GausianNoiseGen(sigmay), ez=GausianNoiseGen(sigmaez);
    while(x<sigmax/2||y<sigmay/2||ez<sigmaez/2){
      x=GausianNoiseGen(sigmax);
      y=GausianNoiseGen(sigmay);
      ez=GausianNoiseGen(sigmaez);
    }
    offset<<x, y, 0, 0, 0, ez;

    Eigen::Affine3d Toffset = poses_[i]*VectorToAffine3d(offset)*poses_[i].inverse();
    Terror_[i] = Toffset;
    clouds_error_[i]->clear();
    pcl::transformPointCloud( *clouds_[i], *clouds_error_[i], Toffset);
  }

}*/
void AlignmentTester::ReAlignScanFixedOffset(double d, double alpha){
  pos_offset = d;
  angle_offset = alpha;
  AllocateScans();
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_real_distribution<> dis(-M_PI, M_PI);
  for(int i=0;i<clouds_error_.size()&& ros::ok();i++){
    Eigen::Matrix<double,6,1> offset;
    /*double rand_angle = dis(gen);
    double x=d*cos(rand_angle);
    double y=d*sin(rand_angle);
    usleep(10);
    double angle_offset = dis(gen)>0 ? alpha : -alpha;*/
    double rand_angle = dis(gen);
    double x=d*1.0/(sqrt(2));
    double y=d*1.0/(sqrt(2));
    usleep(10);
    double angle_offset  = alpha;

    offset<<x,y,0,0,0,angle_offset;
    //cout<<"realign with offset: "<<offset<<endl;
    Eigen::Affine3d Toffset = poses_[i]*VectorToAffine3d(offset)*poses_[i].inverse();
    Terror_[i] = Toffset;
    offsets_[i] = offset;
    clouds_error_[i]->clear();
    pcl::transformPointCloud( *clouds_[i], *clouds_error_[i], Toffset);
  }

}
void AlignmentTester::ReAlignScansNoOffset(){
  cout<<"No error"<<endl;
  AllocateScans();
  for(int i=0;i<clouds_error_.size()&& ros::ok();i++){
    Terror_[i] = poses_[i];
    clouds_error_[i]->clear();
    *clouds_error_[i] = *clouds_[i];
  }

}






}
