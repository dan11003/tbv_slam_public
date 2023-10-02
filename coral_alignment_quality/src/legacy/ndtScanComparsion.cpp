    #include "alignment_checker/ndtScanComparsion.h"


  namespace alignment_checker{

  NdtScanComparsion::NdtScanComparsion(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, double radius, bool downsample, m_type type)
  {
    type_ = type;
    radius_ = radius;
    SetInput(src, target);
    if(type_==rel_ndtp2d){
      double tar_tot_selfscore = 0, src_tot_selfscore = 0;
      int tar_selfoverlap = 0, src_selfoverlap = 0;
      double tar_totlg_det, src_totlg_det;
      (void) CheckAlignment(tar_tot_selfscore, tar_selfoverlap, tar_totlg_det, target_, ndttar_);
      (void) CheckAlignment(src_tot_selfscore, src_selfoverlap, src_totlg_det, src_, ndtsrc_);
      separate_ = (tar_totlg_det+src_totlg_det)/(src_selfoverlap+tar_selfoverlap);
      //separate_ = (tar_tot_selfscore + src_tot_selfscore)/(src_selfoverlap+tar_selfoverlap);
      cout<<"rel ndt sep: "<<separate_<<endl;

      double totsrc, tottar;
      int overlapsrc, overlaptar;
      double totlgdet;
      aligned_ = CheckAlignment(totsrc, overlapsrc, totlgdet, src_, ndttar_);
      aligned_ = CheckAlignment(tottar, overlaptar, totlgdet, target_, ndtsrc_);
      merged_ = (totsrc + tottar)/(overlapsrc+overlaptar);
      cout<<"rel ndt mer: "<<merged_<<endl;
    }
    else{
      separate_ = 0;
      double totsrc;
      int overlap;
      double totlgdet;
      aligned_ = CheckAlignment(totsrc, overlap, totlgdet , src_, ndttar_);
      merged_ = totsrc/overlap;
    }


    delete ndttar_;
    delete ndtsrc_;

  }
  void NdtScanComparsion::SetInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target){


    *src_ = *src;
    *target_ = *target;
    pcl::PointXYZ pref = target->points[0];
    ndttar_ = new perception_oru::NDTMap(new perception_oru::LazyGrid(radius_));
    ndttar_->initialize(pref.x,pref.y,pref.z, 200, 200, 25);
    ndttar_->addPointCloudSimple(*target_, 100);
    ndttar_->computeNDTCells();

    pcl::PointXYZ psrc = src->points[0];
    ndtsrc_ = new perception_oru::NDTMap(new perception_oru::LazyGrid(radius_));
    ndtsrc_->initialize(pref.x,pref.y,pref.z, 200, 200, 25);
    ndtsrc_->addPointCloudSimple(*src_, 100);
    ndtsrc_->computeNDTCells();



  }


  bool NdtScanComparsion::CheckAlignment(double &tot_distance, int &overlap, double &tot_lgdet, pcl::PointCloud<pcl::PointXYZ>::Ptr &scan, perception_oru::NDTMap* reference){
    double integral = 0.1;
    double outlier_ratio = 0.01;
    //outlier_ratio = 0.5;
    double support_size = 4; //???
    double lfc1 = (1-outlier_ratio)/integral;
    double lfc2 = outlier_ratio/pow(support_size,3);
    double lfd3 = -log(lfc2);
    double lfd1 = -log( lfc1 + lfc2 ) - lfd3;
    double lfd2 = -log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / lfd1);
    tot_distance = 0;
    //compute ndt score and conver
    overlap = 0;
    tot_lgdet = 0;
    for(auto point : scan->points){
      perception_oru::NDTCell* cell;
      reference->getCellForPoint(point, cell, true);
      if(cell==NULL)
        continue;
      Eigen::Matrix3d icov = cell->getInverseCov();
      Eigen::Matrix3d cov  = cell->getCov();
      Eigen::Vector3d mean = cell->getMean();
      Eigen::Vector3d point_eig(point.x, point.y, point.z);
      double l = (point_eig-mean).dot(icov*(point_eig-mean));
      if(l*0 != 0) continue;

      if(l > 120) continue;

      tot_lgdet += log(cov.determinant());
      //log(2*M_PI*exp(1.0)*cov.determinant());
      overlap++;
      double score_here = (lfd1*exp(-lfd2*l/2.0));
      tot_distance +=score_here;
    }

  }




  }

