#include "alignment_checker/DataHandler.h"
namespace CorAlignment {


FiledataHandler::FiledataHandler(const std::string& directory, const std::string& poses, const std::string& data_prefix, int index_first_scan) {

}

std::shared_ptr<PoseScan> FiledataHandler::Next(){

}


RadarRosbagHandler::RadarRosbagHandler(const std::string& rosbag_path, const PoseScan::Parameters& scanPars, const int rosbag_offset, const double min_distance, const std::string& gt_topic, const std::string& radar_topic): scanPars_(scanPars), min_distance_(min_distance){
    cout<<"Loading bag file: "<<rosbag_path<<endl;
    bag_.open(rosbag_path, rosbag::bagmode::Read);
    assert(bag_.isOpen());
    view_image_ = std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery({radar_topic}));
    view_pose_  = std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery({gt_topic}));
    assert(view_image_->size() >0 && view_pose_->size() > 0 );

    m_image_ = std::next(view_image_->begin(),rosbag_offset);
    m_pose_ = std::next(view_pose_->begin(),  rosbag_offset);

    cout<<"images: "<<std::distance(view_image_->begin(),view_image_->end())<<",poses"<<std::distance(view_pose_->begin(),view_pose_->end())<<endl;

    pub_image = nh_.advertise<sensor_msgs::Image>("/Navtech/Polar", 1000);
    pub_odom = nh_.advertise<nav_msgs::Odometry>("/gt", 1000);
}
void RadarRosbagHandler::UnpackImage(sensor_msgs::ImageConstPtr& image_msg){

    cv_bridge::CvImagePtr cv_polar_image;
    //cout<<"encoding: "<<image_msg->encoding<<endl;
    if(image_msg->encoding == sensor_msgs::image_encodings::MONO8){
        cv_polar_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
        cv_polar_image->image.convertTo(cv_polar_image->image, CV_8UC1);
        rotate(cv_polar_image->image, cv_polar_image->image, cv::ROTATE_90_COUNTERCLOCKWISE);
        image_msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1;
        //cout<<cv_polar_image->image.rows<<","<<cv_polar_image->image.cols<<endl;
    }
    else
        cv_polar_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC1);


    assert(cv_polar_image != NULL);
    cv_polar_image->header.stamp =  image_msg->header.stamp;
    radar_stream_.push_back(cv_polar_image);
    //cout<<"image timestamp: "<<cv_polar_image->header.stamp.toSec()<<", size: "<<cv_polar_image->image.cols<<", "<<cv_polar_image->image.rows<<endl;

    pub_image.publish(*image_msg);//Publish
}
void RadarRosbagHandler::UnpackPose(nav_msgs::Odometry::ConstPtr& odom_msg){
    nav_msgs::Odometry msg_odom = *odom_msg;
    poseStamped stamped_gt_pose = std::make_pair(Eigen::Affine3d::Identity(), odom_msg->header.stamp);
    tf::poseMsgToEigen(msg_odom.pose.pose, stamped_gt_pose.first);
    pose_stream_.push_back(stamped_gt_pose);

    msg_odom.header.stamp = ros::Time::now(); // Publish
    msg_odom.header.frame_id = "world";
    pub_odom.publish(msg_odom);

}
std::shared_ptr<PoseScan> RadarRosbagHandler::Next(){
    //cout<<"RadarRosbagHandler::Next(){"<<endl;
    ros::Time t0 = ros::Time::now();


    while (m_pose_!=view_pose_->end() && m_image_!=view_image_->end() && ros::ok()) {
        //Reads a new pair
        sensor_msgs::ImageConstPtr image_msg  = m_image_->instantiate<sensor_msgs::Image>();
        nav_msgs::Odometry::ConstPtr odom_msg = m_pose_->instantiate<nav_msgs::Odometry>();
        assert(image_msg != NULL && odom_msg != NULL); //

        //convert and pushes the new pair

        UnpackImage(image_msg);
        UnpackPose(odom_msg);


        // Runs until pose/image streams are synchronized
        while(!synced_){
            if ( (pose_stream_.back().second - radar_stream_.back()->header.stamp) < ros::Duration(0.00001) )
                synced_ = true;
            else if(pose_stream_.back().second.toSec() > image_msg->header.stamp.toSec()){
                radar_stream_.erase(radar_stream_.begin());

                sensor_msgs::ImageConstPtr image_msg_synced  = (++m_image_)->instantiate<sensor_msgs::Image>();
                UnpackImage(image_msg_synced);
            }
            else if(pose_stream_.back().second.toSec() < image_msg->header.stamp.toSec()){
                pose_stream_.erase(pose_stream_.begin());
                nav_msgs::Odometry::ConstPtr odom_msg_synced = (++m_pose_)->instantiate<nav_msgs::Odometry>();
                UnpackPose(odom_msg_synced);
            }
        }
        m_pose_++;
        m_image_++;
        if(pose_stream_.size() < 3)
            continue;
        else if(pose_stream_.size()>3){
            pose_stream_.erase(pose_stream_.begin());
            radar_stream_.erase(radar_stream_.begin());
        }
        //At this point, the time stamps of
        //"pose_stream_[i].second" and
        //"radar_stream_[i]->header.stamp" must be the exact same < 0.00001.

        //const Eigen::Affine3d Tmotion = pose_stream_.front().first.inverse()*pose_stream_.back(); //This is more correct but wrong scaling
        //const Eigen::Affine3d Tmotion = pose_stream_[1].first.inverse()*pose_stream_.back().first; //not very good but okay
        const Eigen::Affine3d Tm2 = pose_stream_[0].first;
        const Eigen::Affine3d Tm1 = pose_stream_[1].first;
        const Eigen::Affine3d Tnow = pose_stream_[2].first;
        const Eigen::Affine3d Tmotion = Tm2.inverse()*Tm1;


        if( (TposePrev.translation()-Tnow.translation()).norm() >= min_distance_){
            TposePrev = Tnow;
            ros::Time t1 = ros::Time::now();

            auto scan = RadarPoseScanFactory(scanPars_, radar_stream_[2], Tnow, Tmotion );
            /*
            auto cenptr = PoseScan_S(new Cen2018Radar(scanPars_, radar_stream_[2], Tnow, Tmotion));
            auto cfear = PoseScan_S(new CFEARFeatures(scanPars_, radar_stream_[2], Tnow, Tmotion));
            auto kstrongpeaks = PoseScan_S(new kstrongStructuredRadar(scanPars_, radar_stream_[2], Tnow, Tmotion));
            auto kstrong = PoseScan_S(new kstrongStructuredRadar(scanPars_, radar_stream_[2], Tnow, Tmotion,false));
            auto cart = PoseScan_S(new CartesianRadar(scanPars_, radar_stream_[2], Tnow, Tmotion));

            AlignmentQualityPlot::PublishPoseScan("test_cen",cenptr,Tnow,"cen_frame");
            AlignmentQualityPlot::PublishPoseScan("test_kstrong_12",kstrong,Tnow,"test_kstrong_12");
            AlignmentQualityPlot::PublishPoseScan("test_kstrongpeaks",kstrongpeaks,Tnow,"test_kstrongpeaks");
            AlignmentQualityPlot::PublishPoseScan("test_cfear",cfear,Tnow,"cfear_frame");
            AlignmentQualityPlot::PublishPoseScan("test_cart",cart,Tnow,"cart_frame"); */

            ros::Time t2 = ros::Time::now();
            timing.Document("Create scan",CFEAR_Radarodometry::ToMs(t2-t1));
            //timing.Document("Read rosbag",CFEAR_Radarodometry::ToMs(t1-t0));
            return scan;
        }
        else
            cout<<"skip"<<endl;


    }
    return nullptr;
}

MockupHandler::MockupHandler(){
    for(int i=1 ; i<=9 ; i+=step_resolution){
        pcl::PointXYZI p;
        p.x = i; p.y = i; p.z = i; p.intensity = 100;
        cloud.push_back(p);
    }

}
std::shared_ptr<PoseScan> MockupHandler::Next(){
    //cout<<"Mockup::Next()"<<endl;


    if(step==100)
        return nullptr;
    else{
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        const Eigen::Affine3d Tmotion = Eigen::Affine3d::Identity();
        T.translation()<<(step++)*step_resolution, 0, 0;
        pcl::transformPointCloud(cloud, *transformed, T.inverse()); //transform cloud into frame of T
        return PoseScan_S(new RawLidar(PoseScan::Parameters(), transformed, T, Tmotion));
    }
}

}
