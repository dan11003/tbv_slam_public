#include "alignment_checker/viewer.h"

namespace alignment_checker {
viewer::viewer(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds, std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> &poses, double radius, std::string &directory, measurement_type type, double ent_reject_ratio):nh_("~")
{
  ent_reject_ratio_ = ent_reject_ratio;
  type_ = type;
  directory_ = directory;
  radius_ = radius;
  offset_<<0, 0, 0, 0, 0, 0;
  downsample_ = true;
  poses_ = poses;
  clouds_ = clouds;
  sub_ = nh_.subscribe("/clicked_point", 1000, &viewer::PointSub,this);
  cloud_local_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/cloud_local", 1000);
  covar_pub = nh_.advertise<visualization_msgs::Marker>("/covar_local", 1000);
  KeyboardInputThread();

}
void viewer::PointSub(const geometry_msgs::PointStamped::ConstPtr &msg){
  last_point = *msg;
  pcl::PointXYZ p;
  p.x = msg->point.x;
  p.y = msg->point.y;
  p.z = msg->point.z;
  pcl::PointCloud<pcl::PointXYZ>::Ptr clocal = comp_->GetMergedScan()->GetLocalCloud(p, radius_);
  if(clocal==NULL){
    cout<<"null"<<endl;
    return;
  }

  ros::Time t = ros::Time::now();
  pcl_conversions::toPCL(t, clocal->header.stamp);
  clocal->header.frame_id = "/world";
  std::cout<<"publish local cloud"<<std::endl;
  cloud_local_pub.publish(*clocal);
  Eigen::Matrix3d cov;


  Eigen::Vector4d xyz_centroid;

  pcl::computeMeanAndCovarianceMatrix(*clocal, cov, xyz_centroid);
  int n = clocal->size();
  cov = n/(n-1)*cov;
  double hq = hq = 1/2.0*log(2*M_PI*exp(1.0)*cov.determinant());
  cout<<"hq: "<<hq<<endl;


}
void viewer::SetScans(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr target;
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);

  if(target_idx_+1>=clouds_.size()|| target_idx_ <0 ||clouds_[target_idx_]==NULL ||clouds_[target_idx_+1]==NULL ){
    cerr<<"index out of range"<<endl;
    return;
  }

  target = clouds_[target_idx_];
  Eigen::Affine3d Toffset = poses_[target_idx_+1]*VectorToAffine3d(offset_)*poses_[target_idx_+1].inverse();
  pcl::transformPointCloud(*clouds_[target_idx_+1], *src, Toffset);

  if(type_==ndtp2d){
    //cout<<"Using "<<mtype2string(type_)<<endl;
    comp_ = bstScanComp(new NdtScanComparsion(src, target, radius_, true, type_));
    bool aligned;
    double merg,sep;

    comp_->GetAlignmentQualityExtended(aligned,merg,sep);
    cout<<"merged score is_: "<<merg<<endl;
  }
  else{
    //cout<<"Using "<<mtype2string(type_)<<endl;
    comp_ = bstScanComp(new ScanComparsion(src, target, radius_, downsample_,type_, ent_reject_ratio_));
    vis_.PlotClouds(comp_);
    vis_.PlotPoses(poses_[target_idx_],poses_[target_idx_+1]);
  }

  pcl::PointXYZ p;
  p.x = last_point.point.x;
  p.y = last_point.point.y;
  p.z = last_point.point.z;
  pcl::PointCloud<pcl::PointXYZ>::Ptr clocal = comp_->GetMergedScan()->GetLocalCloud(p, radius_);
  if(clocal==NULL || clocal->size()==0){
    return;
  }

  ros::Time t = ros::Time::now();
  pcl_conversions::toPCL(t, clocal->header.stamp);
  clocal->header.frame_id = "/world";
  std::cout<<"publish local cloud"<<std::endl;
  cloud_local_pub.publish(*clocal);

}

void viewer::KeyboardInputThread(){
  char input=' ';
  std::cout<<"Welcome to the view score tool"<<std::endl;

  do
  {

    bool save = false;
    input=' ';
    std::cin.clear();
    std::cin>>input;
    if(input=='w')     { offset_(0)+=step_; new_input_=true;}
    else if(input=='s'){ offset_(0)-=step_; new_input_=true;}
    else if(input=='a'){ offset_(1)-=step_; new_input_=true;}
    else if(input=='d'){ offset_(1)+=step_; new_input_=true;}
    else if(input=='z'){ offset_(2)-=step_; new_input_=true;}
    else if(input=='x'){ offset_(2)+=step_; new_input_=true;}
    else if(input=='q'){ offset_(5)-=0.01; new_input_=true;}
    else if(input=='e'){ offset_(5)+=0.01; new_input_=true;}
    else if(input=='c'){ target_idx_+=1;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='v'){ target_idx_-=1;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='b'){ target_idx_+=50;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='n'){ target_idx_-=50;cout<<"Target idx: "<<target_idx_<<endl; new_input_=true;}
    else if(input=='f'){ offset_<<0.05/sqrt(2),0.05/sqrt(2),0,0,0,0.005; new_input_=true;}
    else if(input=='r'){ offset_<<0,0,0,0,0,0; new_input_=true;}
    else if(input=='g'){ downsample_=!downsample_; new_input_=true;}
    else if(input=='h'){ save=true;}
    if(new_input_)
      SetScans();
    else if(save){
      if(fabs(offset_(1))+fabs(offset_(2))+fabs(offset_(5))>0)
        comp_->StoreComparsionData(directory_,"viewer_error.csv");
      else
        comp_->StoreComparsionData(directory_,"viewer_aligned.csv");
    }
    ros::spinOnce();


    new_input_ = false;
  }while(input!='t' && ros::ok());
  cout<<"Exit"<<endl;
  exit(0);

}

}
