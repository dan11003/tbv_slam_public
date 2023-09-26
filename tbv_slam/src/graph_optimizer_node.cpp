
//#include "graph_map/graph_map_fuser.h"
#include <ros/ros.h>

//#include <ndt_map/ndt_conversions.h>
//#include "ndt_generic/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>

//#include "graph_map/lidarUtils/lidar_utilities.h"
//#include "graph_map/graph_map_fuser.h"

#include <time.h>
#include <fstream>
#include <cstdio>

//#include "ndt_generic/pcl_utils.h"
#include <pcl_ros/transforms.h>
//#include "ndt_generic/pcl_utils.h"
//#include "graph_map/graphfactory.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
//#include "ndt_generic/motionmodels.h"
//#include "ndt_generic/motion_model_3d.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include "cfear_radarodometry/types.h"
#include "tbv_slam/posegraph.h"
#include "tbv_slam/loopclosure.h"
#include "ros/service.h"
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "std_srvs/SetBool.h"

#include "robust_mapping_custom_msgs/IndexedPoseScan.h"

/** \brief A ROS node which implements scan matching based on ndt
 * \author Daniel adolfsson
 *
 */


using std::string;
using std::cout;
using std::cerr;
using std::endl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace tbv_slam;


typedef sync_policies::ExactTime<pcl::PointCloud<pcl::PointXYZ>, nav_msgs::Odometry> MySyncPolicy;

class GraphOptimization {



public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor
  GraphOptimization(ros::NodeHandle param_nh) : frame_nr_(0){

    PoseGraph::Parameters pose_graph_par;



    param_nh.param("visualize", visualize, true);
    //param_nh.param<std::string>("map_type",map_type_name,"ndt_map");
    param_nh.param<std::string>("corrected_map_topic", corrected_topic_, "/points_corrected");
    param_nh.param<std::string>("registration_type",reg_type_name,"ndt_d2d_reg");

    param_nh.param<bool>("disable_loop_closure", disable_loop,false);
    param_nh.param<bool>("save_graph", save_graph, false);
    param_nh.param<bool>("load_graph", load_graph, false);
    param_nh.param<bool>("save_graph_map", save_graph_map, false);
    param_nh.param<bool>("enable_odom_refinement", enable_odom_refinement, false);
    param_nh.param<bool>("enable_n_scan_refinement", enable_n_scan_refinement_, false);
    param_nh.param<bool>("enable_appearance_loop_closure", enable_appearance_loop_closure, false);
    param_nh.param<bool>("enable_optimization", enable_optimization, true);

    param_nh.param<std::string>("graph_path", graph_path,"");
    param_nh.param<std::string>("world_link", world_link_id, "/map");
    param_nh.param<std::string>("pcd_output_dir", dir_,"");
    param_nh.param<bool>("save_pcd", save_pcd,false);


    param_nh.param<std::string>("points_topic",points_topic,"/lidar_odom_node/registered"); ///topic to wait for point clouds, if available

    std::string lidar_odom_topic;
    param_nh.param<std::string>("lidar_odom_topic", lidar_odom_topic, std::string("/lidar_odometry/lidar_pose_est"));


    if(points_topic=="")
      cerr<<"No topic specified"<<endl;
    else
      cout<<"Optimization node subscrip to pointcloud \nTopic: "<<points_topic<<endl;

    bool loaded = false;
    if( load_graph )
      if( tbv_slam::PoseGraph::LoadSimpleGraph(graph_path, graph, pose_graph_par) || tbv_slam::PoseGraph::LoadGraph(graph_path, graph) )
        loaded = true;
    if(!loaded)
      graph = boost::shared_ptr<tbv_slam::PoseGraph>(new tbv_slam::PoseGraph(pose_graph_par));

    vis = new tbv_slam::PoseGraphVis(graph);
    /*if(!disable_loop){

      if(enable_appearance_loop_closure){
        cout<<"Enable scan context"<<endl;
       appearance_loop_closure = new tbv_slam::ScanContextClosure(graph);
      }

      //loop_closure = new robust_mapping::SimpleDistanceClosure(graph);
    }
  */



    std::cout<<"Started optimization node"<<std::endl;

    pnts_sub = new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ>> (param_nh, points_topic, 1);
    odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(param_nh, lidar_odom_topic, 1);

    cout<<"Sync topics:\n"<<points_topic<<std::endl<<lidar_odom_topic<<endl;

    sync_lo = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *pnts_sub, *odom_sub);
    sync_lo->registerCallback(boost::bind(&GraphOptimization::PointsOdomCallback,this, _1, _2));

    srv = param_nh.advertiseService("force_optimization", &GraphOptimization::forceOptimization, this);
    srv_save = param_nh.advertiseService("save_map", &GraphOptimization::SaveSrvCallback, this);
    pub_idx_scan = param_nh.advertise<robust_mapping_custom_msgs::IndexedPoseScan>("/idx_scan",100);


  }

  bool GenerateMap() {
    /*octomap_creator::OctomapCreator oct;
    oct.CreateMap(graph->GetClouds(), graph->GetPoses(),sensorPose_.inverse());
    oct.SaveMap();

    string time = ndt_generic::currentDateTimeString();
    return true;*/
  }

  void AddNode(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Affine3d &Tpose, Eigen::Matrix<double,6,6> &Cov) {

    static int node = 1;
    static Eigen::Affine3d Tprev = Tpose;
    const Eigen::Affine3d Tprevinv = Tprev.inverse();
    const Eigen::Affine3d Tdiff = Tprevinv*Tpose;


    Eigen::Matrix<double,6,6> cov_rotated = Cov;
    cov_rotated.block<3,3>(0,0) = Tprevinv.rotation()*Cov.block<3,3>(0,0)*Tprevinv.rotation().transpose();
    cov_rotated(0,5) = 0;
    cov_rotated(5,0) = 0;
    cov_rotated(2,2) = 1;
    cov_rotated(3,3) = 1;
    cov_rotated(4,4) = 1;

    cout<<"covar: "<<cov_rotated<<endl;

    std::cout << "PROBLEM; UNCOMMENTED ADD NODE, NEEDS TO BE CHANGED TO INCLUDING TIME" << std::endl;
    /*if(node==1)
      graph->AddNode(Tpose, cloud, cov_rotated, node, node-1);
    else
      graph->AddNode(Tdiff, cloud, cov_rotated, node, node-1);*/


    robust_mapping_custom_msgs::IndexedPoseScan msg;
    pcl::toROSMsg(*cloud, msg.cloudsrc);
    msg.id = node-1;
    //Eigen::Affine3d T = robust_mapping::PoseCeresToEig(graph->GetPoses()[graph->GetPosesSize()-1]);
    //tf::poseEigenToMsg(T, msg.Tsrc);
    pub_idx_scan.publish(msg);
    node++;
    Tprev = Tpose;
  }
  void PlotClouds(pcl::PointCloud<pcl::PointXYZ>& cloud, ros::Time time = ros::Time::now()){
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(cloud, msg_out);
    msg_out.header.frame_id = laser_link_id;
    msg_out.header.stamp = time;
    point2_publisher_.publish(msg_out);
  }


  bool SaveSrvCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& respons) {


    graph->m_graph.lock();
    if(save_graph)
      tbv_slam::PoseGraph::SaveGraph(graph_path, graph);
    /*if(save_pcd)
      graph->SavePointCloud(dir_);*/
    /*if(save_graph_map)
      GenerateMap();*/
    graph->m_graph.unlock();

    return true;
  }
  bool forceOptimization(std_srvs::Empty::Request& request, std_srvs::Empty::Response& respons) {
    graph->ForceOptimize();
    return true;
  }

  void SaveGraph(){

    graph->m_graph.lock();
    if(save_graph)
      tbv_slam::PoseGraph::SaveGraph(graph_path, graph);
    graph->m_graph.unlock();

  }
  void PointsOdomCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg){
    //cout<<"Optimizer recieved cloud:"<<cloud_msg->points.size()<<", odom_msg"<<odom_msg->pose.pose.position<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());// = *cloud_msg;
    cloud->header.stamp = cloud_msg->header.stamp;
    Eigen::Affine3d Tpose;
    tf::poseMsgToEigen(odom_msg->pose.pose, Tpose);
    pcl::transformPointCloud(*cloud_msg, *cloud, Tpose.inverse());

    double cov[36];
    for(int i=0;i<odom_msg->pose.covariance.size();i++){
      cov[i]=odom_msg->pose.covariance[i];
    }
    Eigen::Matrix<double,6,6> Cov = Eigen::Map<Eigen::Matrix<double,6,6> >(cov);

    this->AddNode(cloud, Tpose, Cov);
  }


protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  //pogm::GraphMapFuser *fuser_ = NULL;

  ros::Subscriber pointcloud_callback;

  // Components for publishing
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listener_;
  ros::Publisher pose_publisher;
  Eigen::Affine3d pose_, T, sensorPose_;
  tf::StampedTransform T_odom;


  unsigned int frame_nr_;
  tf::Transform tf_sensor_pose_;
  std::string map_type_name, reg_type_name;
  std::string points_topic="", map_dir;
  bool visualize;
  bool use_imu_constraint;
  std::string world_link_id;
  bool enable_odom_refinement, enable_n_scan_refinement_;


  double sensor_pose_x,sensor_pose_y,sensor_pose_z,sensor_pose_r,sensor_pose_p,sensor_pose_t;
  double sensor_offset_t_;
  string output_pointcloud_topic_name;
  ros::Publisher point2_publisher_, pub_idx_scan;

  std::string laser_link_id;
  std::string odometry_link_id, base_link_id, lidar_odometry_link_id, lidar_base_link_id, imu_parent_link, imu_child_link;
  tbv_slam::PoseGraphVis *vis;
  tbv_slam::loopclosure *loop_closure, *appearance_loop_closure, *odom_refinement, *nscan_refinement;


  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub;
  message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > *pnts_sub;
  message_filters::Synchronizer< MySyncPolicy > *sync_lo;
  ros::ServiceServer srv, srv_save;
  std::string graph_path; //global variable
  boost::shared_ptr<tbv_slam::PoseGraph> graph;
  bool save_graph, load_graph;
  bool disable_loop, enable_appearance_loop_closure;
  std::string dir_, corrected_topic_;
  bool save_pcd;  bool save_graph_map;
  bool enable_optimization;


};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_odom");
  ros::NodeHandle param("~");
  GraphOptimization t(param);
  ros::spin();
  ROS_INFO("Exit graph optimization node");
  t.SaveGraph();
  return 0;
}



