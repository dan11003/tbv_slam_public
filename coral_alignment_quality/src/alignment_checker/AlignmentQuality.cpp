#include "alignment_checker/AlignmentQuality.h"
namespace CorAlignment {



/////**************** CORAL Quality ******************////////

void CorAlRadarQuality::GetNearby(const pcl::PointXY& query, Eigen::MatrixXd& nearby_src, Eigen::MatrixXd& nearby_ref, Eigen::MatrixXd& merged){
    std::vector<int> pointIdxRadiusSearch_src, pointIdxRadiusSearch_ref;
    std::vector<float> pointRadiusSquaredDistance_src, pointRadiusSquaredDistance_ref;
    int nr_nearby_src = kd_src.radiusSearch(query, par_.radius, pointIdxRadiusSearch_src, pointRadiusSquaredDistance_src);
    int nr_nearby_ref = kd_ref.radiusSearch(query, par_.radius, pointIdxRadiusSearch_ref, pointRadiusSquaredDistance_ref);
    nearby_src.resize(nr_nearby_src,2);
    nearby_ref.resize(nr_nearby_ref,2);
    merged.resize(nr_nearby_ref + nr_nearby_src,2);
    int tot = 0;
    for(std::size_t i = 0; i < pointIdxRadiusSearch_src.size (); i++){
        nearby_src(i,0) = src_pcd->points[pointIdxRadiusSearch_src[i]].x;
        nearby_src(i,1) = src_pcd->points[pointIdxRadiusSearch_src[i]].y;
        merged.block<1,2>(tot++,0) = nearby_src.block<1,2>(i,0);
    }
    for(std::size_t i = 0; i < pointIdxRadiusSearch_ref.size (); i++){
        nearby_ref(i,0) = ref_pcd->points[pointIdxRadiusSearch_ref[i]].x;
        nearby_ref(i,1) = ref_pcd->points[pointIdxRadiusSearch_ref[i]].y;
        merged.block<1,2>(tot++,0) = nearby_ref.block<1,2>(i,0);
    }
}
bool CorAlRadarQuality::Covariance(Eigen::MatrixXd& x, Eigen::Matrix2d& cov, Eigen::Vector2d& mean){ //mean already subtracted from x
    //Compute and subtract mean
    //cout<<"before subtract: "<< x<<endl;
    //cout<<"mean: "<<x.rowwise().mean().transpose()<<endl;
    if(x.rows() <= 2)
        return false;

    mean = x.colwise().mean();

    for(int i=0;i<x.rows();i++) // subtract mean
        x.block<1,2>(i,0) = x.block<1,2>(i,0) - mean.transpose();

    Eigen::Matrix2d covSum = x.transpose()*x;
    float n = x.rows();
    cov = covSum*1.0/(n-1.0);
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2,2>(0,0));
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    //cout<<"cond "<<cond<<endl;

    return true;//cond < 100000;
}
bool CorAlRadarQuality::ComputeKLDiv(const Eigen::Vector2d& u0, const Eigen::Vector2d& u1, const Eigen::Matrix2d& S0, const Eigen::Matrix2d& S1, const int index){

    //return  (u1-u0).norm();

    Eigen::Matrix2d S1i = S1.inverse();
    Eigen::Matrix2d S1iS0 = S1i*S0;
    double S1iS0_trace = S1iS0.trace();
    double mahal = (u1-u0).transpose()*S1i*(u1-u0);
    double k = 3;
    double d1 = S1.determinant();
    double d0 = S0.determinant();
    double logdetratio= log(d1/d0);

    double score = 1/2.0*(S1iS0_trace+mahal-k+logdetratio);

    Eigen::Matrix2d C = S0+S1;
    Eigen::Vector2d err = u1-u0;
    //double score = err.transpose()*C.inverse()*err;

    bool score_problem = isnan(score) || !isfinite(score);
    joint_res_[index] = score_problem ? 0 : score;
    sep_res_[index] = 0;
    return score;

}

bool CorAlRadarQuality::ComputeEntropy(const Eigen::Matrix2d& cov_sep, const Eigen::Matrix2d& cov_joint, int index){

    double det_j = cov_joint.determinant();
    double det_s = cov_sep.determinant();
    if(isnanl(det_s) || isnanl(det_j))
        return false;

    const double sep_entropy =  1.0/2.0*log(2.0*M_PI*exp(1.0)*det_s+0.00000001);
    const double joint_entropy = 1.0/2.0*log(2.0*M_PI*exp(1.0)*det_j+0.00000001);

    if( isnan(sep_entropy) || isnan(joint_entropy) )
        return false;
    else{
        sep_res_[index] = sep_entropy;
        joint_res_[index] = joint_entropy;
        return true;
    }
}
CorAlRadarQuality::CorAlRadarQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset)  : AlignmentQuality(src, ref, par, Toffset)
{
    //CFEAR_Radarodometry::Document()

    ros::Time t0 = ros::Time::now();
    auto src_kstrong_structured = std::dynamic_pointer_cast<kstrongStructuredRadar>(src);
    auto ref_kstrong_structured = std::dynamic_pointer_cast<kstrongStructuredRadar>(ref);

    auto src_pcd_entropy = src->GetCloudCopy(src->GetAffine()*Toffset);
    auto ref_pcd_entropy = ref->GetCloudCopy(ref->GetAffine());


    assert(ref_pcd_entropy != NULL && src_pcd_entropy !=NULL);

    std::vector<double> src_i,ref_i;

    src_pcd = pcl3dto2d(src_pcd_entropy, src_i);
    ref_pcd = pcl3dto2d(ref_pcd_entropy, ref_i);
    sep_intensity_.insert( sep_intensity_.end(),src_i.begin(),src_i.end() );
    sep_intensity_.insert( sep_intensity_.end(),ref_i.begin(),ref_i.end() );

    kd_src.setInputCloud(src_pcd);
    kd_ref.setInputCloud(ref_pcd);

    const size_t merged_size = src_pcd->size() + ref_pcd->size();
    assert(src_pcd->size() > 0 && ref_pcd->size()>0);


    sep_res_.resize(merged_size, 100.0);
    sep_valid.resize(merged_size, false);
    joint_res_.resize(merged_size, 100.0);
    diff_res_.resize(merged_size, 100.0);
    covs_sep_.resize(merged_size);
    cov_joint_.resize(merged_size);
    means_sep_.resize(merged_size);
    means_joint_.resize(merged_size);

    ros::Time t1 = ros::Time::now();
    int index = -1;
    for (auto && searchPoint : src_pcd->points){
        index++;
        Eigen::MatrixXd msrc, mref, mjoint;
        GetNearby(searchPoint, msrc, mref, mjoint);
        if(mref.rows() < overlap_req_)
            continue;

        if( Covariance(msrc, covs_sep_[index], means_sep_[index]) && Covariance(mjoint, cov_joint_[index], means_joint_[index]) ){
            if(par.ent_cfg == AlignmentQuality::parameters::kl)
            {
                sep_valid[index] = ComputeKLDiv(means_sep_[index], means_sep_[index], covs_sep_[index], covs_sep_[index], index);
            }
            else
            {
                sep_valid[index] = ComputeEntropy(covs_sep_[index], cov_joint_[index], index);
            }
            //sep_valid[index] = ComputeKLDiv(means_sep_[index], means_joint_[index], covs_sep_[index], cov_joint_[index], index);

            //bool KLDiv(const Eigen::Vector3d& u0, const Eigen::Vector3d& u1, const Eigen::Matrix3d& S0, const Eigen::Matrix3d& S1, const int index);
        }
    }

    for (auto && searchPoint : ref_pcd->points){
        index++;
        Eigen::MatrixXd msrc, mref, mjoint;
        GetNearby(searchPoint, msrc, mref, mjoint);
        if(msrc.rows() < overlap_req_){
            continue;
        }

        if( Covariance(mref, covs_sep_[index], means_sep_[index]) && Covariance(mjoint, cov_joint_[index],means_joint_[index]) ){
            if(par.ent_cfg == AlignmentQuality::parameters::kl)
            {
                sep_valid[index] = sep_valid[index] = ComputeKLDiv(means_sep_[index], means_joint_[index], covs_sep_[index], cov_joint_[index], index);
            }
            else
            {
                sep_valid[index] = ComputeEntropy(covs_sep_[index], cov_joint_[index], index);
            }
        }
    }
    ros::Time t2 = ros::Time::now();
    for (int i=0;i<sep_res_.size();i++){
        if(sep_valid[i]){
            const double w = par.weight_res_intensity ? sep_intensity_[i] : 1.0;
            w_sum_ += w;
            joint_res_[i] = w*joint_res_[i];
            sep_res_[i] = w*sep_res_[i];
            diff_res_[i] = joint_res_[i] - sep_res_[i];
            joint_ += joint_res_[i]; sep_ += sep_res_[i];
            count_valid++;
        }
    }


    if(count_valid > 0){
        sep_ /=w_sum_;
        joint_ /=w_sum_;
        diff_ = joint_ - sep_;
    }

    overlap_  = count_valid/((double)merged_size);
    //cout  << "overlap : " << overlap_ << ", merged size: " << merged_size << ", count_valid: " << count_valid <<  endl;
    if (overlap_ < 0.1){
        quality_ = {joint_, sep_, (par.output_overlap ? overlap_ : 0.0)}; //
        valid_ = false;
    }
    else {
        quality_ = {joint_, sep_,  (par.output_overlap ? overlap_ : 0.0)}; // (par.output_overlap ? overlap_ : 0.0)
        valid_ = true;
    }
    //cout << "Q: " << quality_ << endl;

    //cout << "sep: "  << sep_ << endl;
    //cout << "joint_: " << joint_ << endl;
    //cout << "w_sum_: " << w_sum_ << endl;
    //cout << "overlap: " << overlap << endl;

    //Assign entrpy values
    index = 0;
    for (auto && p : src_pcd_entropy->points)  // asign entropy
        p.intensity = sep_res_[index]; // set intensity
    for (auto && p : ref_pcd_entropy->points)  // asign entropy
        p.intensity = sep_res_[index++]; // set intensity

    merged_entropy = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    *merged_entropy += *src_pcd_entropy;
    *merged_entropy += *ref_pcd_entropy;

    for (int i = 0; i < merged_entropy->size();i++)
        merged_entropy->points[i].intensity = diff_res_[i]; // set intensity
    if(par_.visualize){
        AlignmentQualityPlot::PublishCloud("/coral_src",    src_pcd_entropy, Eigen::Affine3d::Identity(), "coral_world");
        AlignmentQualityPlot::PublishCloud("/coral_ref",    ref_pcd_entropy, Eigen::Affine3d::Identity(), "coral_world");
        AlignmentQualityPlot::PublishCloud("/coral_merged", merged_entropy,  Eigen::Affine3d::Identity(), "coral_world");
    }
    ros::Time t3 = ros::Time::now();
}


/////**************** End of CORAL Quality ******************////////


std::vector<double> p2pQuality::GetQualityMeasure(){

    //Calculate the mean of all the residuals
    int residuals_size = residuals_.size();
    if(residuals_size  == 0)
        return {0, 0, 0};

    double means = 0;
    for(int i=0; i<residuals_size; i++){
        means += residuals_[i];
    }
    means = means/double(residuals_size);
    return {means, 0, 0};
}



p2pQuality::p2pQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset) : AlignmentQuality(src, ref, par, Toffset){


    //pcl::transform



    //Transform into "world" frame and than into frame of "ref"
    const Eigen::Affine3d Tsrc = src->GetAffine();
    const Eigen::Affine3d Tref = ref->GetAffine();
    const Eigen::Affine3d Tchange = Tref.inverse()*Tsrc*Toffset;
    //cout<<"change\n"<<Tchange.matrix()<<endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cld = src->GetCloudCopy(Tchange);//get cloud and also change reference frame to ref
    pcl::PointCloud<pcl::PointXYZI>::Ptr ref_cld = ref->GetCloudNoCopy();
    //cout<<"src: "<<src_cld->size()<<", ref: "<<ref_cld->size()<<endl;
    /*for(auto && p : src_cld->points)
    cout<<"src: "<<p.getArray3fMap().transpose()<<endl;
  for(auto && p : ref_cld->points)
    cout<<"ref: "<<p.getArray3fMap().transpose()<<endl;*/

    //Build kd tree for fast neighbor search
    kdtree_.setInputCloud(ref_cld); // ref cloud

    //Calculate the sqaured distances for all 'p' in the point cloud

    for(auto && p : src_cld->points){
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<int> pointIdxRadiusSearch;
        const double radius = par_.radius;
        kdtree_.setSortedResults(true);

        if ( kdtree_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
            residuals_.push_back(pointRadiusSquaredDistance[0]);
        }
    }
    //cout<<"residuals: "<<residuals_.size()<<endl;


}


keypointRepetability::keypointRepetability(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset) : AlignmentQuality(src, ref, par, Toffset){

    //Transform into "world" frame and than into frame of "ref"
    const Eigen::Affine3d Tsrc = src->GetAffine();
    const Eigen::Affine3d Tref = ref->GetAffine();
    const Eigen::Affine3d Tchange = Tref.inverse()*Tsrc*Toffset;
    //cout<<"change\n"<<Tchange.matrix()<<endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cld = src->GetCloudCopy(Tchange);//get cloud and also change reference frame to ref
    pcl::PointCloud<pcl::PointXYZI>::Ptr ref_cld = ref->GetCloudNoCopy();

    kdtree_ref_.setInputCloud(ref_cld); // ref cloud
    //kdtree_src_.setInputCloud(src_cld); // ref cloud

    //Calculate the sqaured distances for all 'p' in the point cloud
    double absolute_repeatable_keypoints = 0.0; // within a tolerance radius
    for(auto && p : src_cld->points){
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<int> pointIdxRadiusSearch;
        const double radius = par_.radius;
        kdtree_ref_.setSortedResults(true);

        if ( kdtree_ref_.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
            //residuals_.push_back(pointRadiusSquaredDistance[0]);
            absolute_repeatable_keypoints ++;
        }
    }
    const double nr_keypoints = src_cld->points.size();
    const double relative_repeatable_keypoints = absolute_repeatable_keypoints/nr_keypoints;
    quality_ = {relative_repeatable_keypoints, absolute_repeatable_keypoints, nr_keypoints};
    //cout<<quality_<<endl;

    //quality_ = {count_nearby, (double)src_cld->points.size(), 0.0};
    //cout<<"residuals: "<<residuals_.size()<<endl;


}

CFEARQuality::CFEARQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset)  : AlignmentQuality(src, ref, par, Toffset){
    auto CFEAR_src = std::dynamic_pointer_cast<CFEARFeatures>(src);
    auto CFEAR_ref = std::dynamic_pointer_cast<CFEARFeatures>(ref);
    assert(CFEAR_src!=NULL && CFEAR_ref!=NULL);
    CFEAR_Radarodometry::costmetric pnt_cost = CFEAR_Radarodometry::Str2Cost(par.method);

    CFEAR_Radarodometry::n_scan_normal_reg reg(pnt_cost, CFEAR_Radarodometry::losstype::Huber, 0.3);
    std::vector<CFEAR_Radarodometry::MapNormalPtr> feature_vek = {CFEAR_ref->CFEARFeatures_, CFEAR_src->CFEARFeatures_};
    std::vector<Eigen::Affine3d> Tvek = {CFEAR_ref->GetAffine(),CFEAR_src->GetAffine()*Toffset};
    if(par_.visualize){
        CFEAR_Radarodometry::MapPointNormal::PublishMap("scan1", feature_vek[0], Tvek[0], "world", 1 );
        CFEAR_Radarodometry::MapPointNormal::PublishMap("scan2", feature_vek[1], Tvek[1], "world", -1);
    }
    double score = 0;
    if(reg.GetCost(feature_vek, Tvek, score, residuals_)){
        const double tot_size = (CFEAR_src->CFEARFeatures_->GetSize() + CFEAR_ref->CFEARFeatures_->GetSize()) / 2.0;
        const double overlap = (double)residuals_.size();
        quality_ = {score, (double)residuals_.size(), tot_size};
        valid_ = true;
    }else{
        quality_ = {0, 0, 0};
        valid_ = true;
    }
    //cout << quality_ << endl;
}

CorAlCartQuality::CorAlCartQuality(std::shared_ptr<PoseScan> ref, std::shared_ptr<PoseScan> src,  const AlignmentQuality::parameters& par, const Eigen::Affine3d Toffset)  : AlignmentQuality(src, ref, par, Toffset){

    auto rad_cart_src = std::dynamic_pointer_cast<CartesianRadar>(src);
    auto rad_cart_ref = std::dynamic_pointer_cast<CartesianRadar>(ref);

    assert(rad_cart_src != nullptr || rad_cart_ref != nullptr);
    assert (rad_cart_src->ToString() == "CartesianRadar");
    const Eigen::Affine3d Tsrc = rad_cart_src->GetAffine();
    const Eigen::Affine3d Tref = rad_cart_src->GetAffine();
    const Eigen::Affine3d Tchange = Tref.inverse()*Tsrc*Toffset;


    auto cart_src_transformed = CreateImage(rad_cart_src->cart_);
    RotoTranslation(rad_cart_src->cart_->image, cart_src_transformed->image, Tchange,rad_cart_ref->cart_resolution_);

    cv_bridge::CvImagePtr Absdiff = CreateImage(rad_cart_src->cart_);
    cv::absdiff(cart_src_transformed->image, rad_cart_ref->cart_->image, Absdiff->image);
    if(par_.visualize){
        AlignmentQualityPlot::PublishRadar("/cart_transformed", cart_src_transformed);
        AlignmentQualityPlot::PublishRadar("/cart_transformed", cart_src_transformed);
        AlignmentQualityPlot::PublishRadar("/image_diff", Absdiff);
    }
    const double abs_diff = cv::sum(Absdiff->image)[0];
    quality_ = {abs_diff, 0,0};
    cout<<"abs_diff: "<<abs_diff<<endl;

    if(par_.visualize){
        AlignmentQualityPlot::PublishPoseScan("/radar_src", src, src->GetAffine()*Toffset, "/src_link");
        AlignmentQualityPlot::PublishPoseScan("/radar_ref", ref, ref->GetAffine()*Toffset, "/ref_link");
    }
}






/////**************** Publishing tools ******************////////
std::map<std::string, ros::Publisher> AlignmentQualityPlot::pubs = std::map<std::string, ros::Publisher>();


void AlignmentQualityPlot::PublishRadar(const std::string& topic, cv_bridge::CvImagePtr& img, const Eigen::Affine3d& T, const std::string& frame_id, const ros::Time& t ){
    std::map<std::string, ros::Publisher>::iterator it = AlignmentQualityPlot::pubs.find(topic);
    if(it == pubs.end()){
        ros::NodeHandle nh("~");
        pubs[topic] = nh.advertise<sensor_msgs::Image>(topic,1000);
        it = AlignmentQualityPlot::pubs.find(topic);
    }

    cv_bridge::CvImagePtr img_tmp = boost::make_shared<cv_bridge::CvImage>();
    img_tmp->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    img_tmp->header = img->header;
    img->image.convertTo(img_tmp->image, CV_8UC1, 255.0);

    img_tmp->header.stamp = t;
    img_tmp->header.frame_id = frame_id;
    sensor_msgs::Image imsg;
    img_tmp->toImageMsg(imsg);
    pubs[topic].publish(imsg);
    if(!frame_id.empty())
        PublishTransform(T,frame_id);

}

void AlignmentQualityPlot::PublishPoseScan(const std::string& topic, std::shared_ptr<PoseScan>& scan_plot, const Eigen::Affine3d& T, const std::string& frame_id, const int value){
    if(scan_plot == NULL)
        return;

    auto cart_rad = std::dynamic_pointer_cast<CartesianRadar>(scan_plot);
    if(cart_rad != NULL){
        cart_rad->polar_filtered_;
        PublishRadar(topic+"_polar",cart_rad->polar_,T,frame_id);
        PublishRadar(topic+"_cart", cart_rad->cart_,T,frame_id);
    }
    else{
        pcl::PointCloud<pcl::PointXYZI>::Ptr cld_plot = scan_plot->GetCloudNoCopy(); //Extract point cloud from PoseScan
        PublishCloud(topic,cld_plot,T,frame_id,value);
    }
    auto cfear = std::dynamic_pointer_cast<CFEARFeatures>(scan_plot);
    if(cfear  != NULL){
        Eigen::Affine3d Tnc = T;
        CFEAR_Radarodometry::MapPointNormal::PublishMap("CFEARFatures",cfear->CFEARFeatures_,Tnc,"world",-1);
    }
}

void AlignmentQualityPlot::PublishCloud(const std::string& topic, pcl::PointCloud<pcl::PointXYZI>::Ptr& cld_plot, const Eigen::Affine3d& T, const std::string& frame_id, const int value){

    std::map<std::string, ros::Publisher>::iterator it = AlignmentQualityPlot::pubs.find(topic);
    if(it == pubs.end()){
        ros::NodeHandle nh("~");
        pubs[topic] = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic,1000);
    }
    ros::Time t = ros::Time::now();
    pcl_conversions::toPCL(t, cld_plot->header.stamp);
    cld_plot->header.frame_id = frame_id;
    pubs[topic].publish(*cld_plot);
    PublishTransform(T,frame_id);
}

void AlignmentQualityPlot::PublishTransform(const Eigen::Affine3d& T, const std::string& frame_id, const ros::Time& t){
    static tf::TransformBroadcaster Tbr;
    tf::Transform Tf;
    std::vector<tf::StampedTransform> trans_vek;
    tf::transformEigenToTF(T, Tf);
    trans_vek.push_back(tf::StampedTransform(Tf, t, "/world", frame_id));
    Tbr.sendTransform(trans_vek);
}

/* INTERFACE */

ros::Publisher AlignmentQualityInterface::pub_train_data;
std::vector<std::vector<double>> AlignmentQualityInterface::training_data_;
std::vector<std::vector<double>> AlignmentQualityInterface::vek_perturbation_ = AlignmentQualityInterface::CreatePerturbations();

std::vector<bool> AlignmentQualityInterface::TrainingDataService(PoseScan_S& scan_current, PoseScan_S& scan_loop)
{
    ros::NodeHandle nh("~");
    ros::ServiceClient client = nh.serviceClient<alignment_checker::AlignmentData>("/alignment_service");
    std::vector<bool> aligned_results;

    for(auto && verr : vek_perturbation_)
    {
        AlignmentQuality::parameters quality_par;
        quality_par.method = "Coral";
        const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);

        AlignmentQuality_S quality = AlignmentQualityFactory::CreateQualityType(scan_current, scan_loop, quality_par, Tperturbation);

        double sum = 0;
        for(auto && e : verr)
            sum+=fabs(e);
        bool aligned = sum < 0.0001;
        std::cout << aligned << ", " << sum << std::endl;

        std::vector<double> quality_measure = quality->GetQualityMeasure();
        std_msgs::Float64MultiArray training_data;
        training_data.data = {(double)aligned, quality_measure[0], quality_measure[1], quality_measure[2]};

        alignment_checker::AlignmentData srv;
        srv.request.score = training_data.data;

        if(client.call(srv))
            aligned_results.push_back((double)srv.response.aligned);
        else
            ROS_ERROR("Failed to call service alignment_service");
    }
    return aligned_results;
}

double AlignmentQualityInterface::AlignmentDataService(PoseScan_S& ref, PoseScan_S& src)
{
    ros::NodeHandle nh("~");
    ros::ServiceClient client = nh.serviceClient<alignment_checker::AlignmentData>("/alignment_service");
    
    AlignmentQuality::parameters quality_par;
    quality_par.method = "Coral";

    AlignmentQuality_S quality = AlignmentQualityFactory::CreateQualityType(ref, src, quality_par);

    std::vector<double> quality_measure = quality->GetQualityMeasure();
    std_msgs::Float64MultiArray alignment_data;
    alignment_data.data = {1.0, quality_measure[0], quality_measure[1], quality_measure[2]};

    alignment_checker::AlignmentData srv;
    srv.request.score = alignment_data.data;

    double aligned_result = 0;
    if(client.call(srv))
        aligned_result = (double)srv.response.aligned;
    else
        ROS_ERROR("Failed to call service alignment_service");

    return aligned_result;
}


void AlignmentQualityInterface::SaveTrainingData(const std::string& training_data_sequence)
{   
    std::ofstream result_file;
    std::string training_data_path = std::getenv("BAG_LOCATION") + (std::string)"/place_recognition_eval/training_data/";

    // Create subdirectory if it doesnt exists
    if (!boost::filesystem::exists(training_data_path))
        boost::filesystem::create_directory(training_data_path);

    result_file.open(training_data_path + training_data_sequence, std::ofstream::out);
    result_file << "aligned,score1,score2,score3" << std::endl;

    for(auto && data : AlignmentQualityInterface::training_data_)
    {
        result_file << data[3] << "," << data[0] << "," << data[1] << "," << data[2] << std::endl;
    }
    result_file.close();

    std::cout << "Saved training data in " << training_data_path + training_data_sequence << std::endl;
}

void AlignmentQualityInterface::UpdateTrainingData(PoseScan_S& ref, PoseScan_S& src, const bool visualize)
{   
    for(auto && verr : AlignmentQualityInterface::vek_perturbation_)
    {
        AlignmentQuality::parameters quality_par;
        quality_par.method = "Coral";
        const Eigen::Affine3d Tperturbation = VectorToAffine3dxyez(verr);
        //std::cout << Tperturbation.translation().transpose() << std::endl;
        AlignmentQuality_S quality = AlignmentQualityFactory::CreateQualityType(ref, src, quality_par, Tperturbation);

        double sum = 0;
        for(auto && e : verr)
            sum+=fabs(e);
        bool aligned = sum < 0.0001;

        std::vector<double> quality_measure = quality->GetQualityMeasure();
        quality_measure.push_back((double)aligned);
        AlignmentQualityInterface::training_data_.push_back(quality_measure);
        // cout << aligned << ", " << quality_measure[0] << "," << quality_measure[1] << "," << quality_measure[2] << std::endl;

        if (visualize)
        {
            // Publish frame for radar
            tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::transformEigenToTF(src->GetAffine() * Tperturbation, transform);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Tperturbation_frame"));

            // Publish pertubation point cloud
            auto src_cloud = src->GetCloudCopy(src->GetAffine() * Tperturbation);

            CorAlignment::AlignmentQualityPlot::PublishCloud("/point_cloud_source_Tper", src_cloud, Eigen::Affine3d::Identity(), "coral_world");

        }
    }
}


const std::vector<std::vector<double>> AlignmentQualityInterface::CreatePerturbations()
{
    // Ground truth
    std::vector<std::vector<double>> vek_perturbation;
    vek_perturbation.push_back({0,0,0});
    
    double range_error = 0.5;
    //double theta_range = 2*M_PI;
    // double theta_range = 2*M_PI;
    //int offset_rotation_steps = 4;
    //double theta_error = 0.0; //0.57*M_PI/180.0;

    // Induce errors
    vek_perturbation = {
        {0,0,0}, // Aligned
        {range_error, 0, 0}, {0, range_error, 0}, {-range_error, 0, 0}, {0, -range_error, 0}  //Missaligned
    };


    return vek_perturbation;
}

}

