
#include "graph_map/graph_map_fuser.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ndt_map/ndt_conversions.h>
#include "ndt_generic/utils.h"
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

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#include <ndt_map/NDTMapMsg.h>

#include "graph_map/lidarUtils/lidar_utilities.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <time.h>
#include <fstream>
#include <cstdio>
#include "velodyne_msgs/VelodyneScan.h"
#include <velodyne_pointcloud/rawdata.h>
#include "ndt_generic/pcl_utils.h"


#ifndef SYNC_FRAMES
#define SYNC_FRAMES 20
#define MAX_TRANSLATION_DELTA 2.0
#define MAX_ROTATION_DELTA 0.5
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */
using namespace perception_oru;
using namespace graph_map;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> LaserPoseSync;
typedef message_filters::sync_policies::ApproximateTime<velodyne_msgs::VelodyneScan, nav_msgs::Odometry> PointsOdomSync;
typedef message_filters::sync_policies::ApproximateTime<velodyne_msgs::VelodyneScan, nav_msgs::Odometry> PointsGTOdomSync;
typedef message_filters::sync_policies::ApproximateTime<velodyne_msgs::VelodyneScan, geometry_msgs::PoseStamped> PointsPoseSync;

class GraphMapFuserNode {

protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  GraphMapFuser *fuser_;
  message_filters::Subscriber<velodyne_msgs::VelodyneScan> *points2_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
  plotmarker plot_marker;

  message_filters::Subscriber<nav_msgs::Odometry> *gt_fuser_sub_;
  ros::Subscriber gt_sub,points2OdomTfSub;

  // Components for publishing
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listener_;
  ros::Publisher output_pub_;
  Eigen::Affine3d pose_, T, sensorPose_;


  unsigned int frame_nr_;
  double varz;
  tf::Transform tf_sensor_pose_;
  std::string map_type_name,reg_type_name;
  std::string map_name="graph_map";
  std::string points_topic="", map_dir, odometry_topic,odometry_adjusted_topic;
  std::string file_format_map=".JFF";
  std::string world_link_id, odometry_link_id, fuser_base_link_id,laser_link_id, init_pose_frame, gt_topic, bag_name,state_base_link_id;
  double size_x, size_y, size_z, resolution, max_range, min_range;
  bool visualize, match2D, matchLaser, beHMT, useOdometry,
  initPoseFromGT, initPoseFromTF, initPoseSet, gt_mapping;
  string velodyne_config_file;
  double pose_init_x,pose_init_y,pose_init_z,
  pose_init_r,pose_init_p,pose_init_t;
  double sensor_pose_x,sensor_pose_y,sensor_pose_z,
  sensor_pose_r,sensor_pose_p,sensor_pose_t;
  double sensor_offset_t_;

  laser_geometry::LaserProjection projector_;
  message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
  message_filters::Synchronizer< LaserPoseSync > *sync_lp_;

  message_filters::Synchronizer< PointsGTOdomSync > *sync_GTodom_;
  message_filters::Synchronizer< PointsOdomSync > *sync_po_;
  message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
  ros::ServiceServer save_map_;
  ros::Time time_now,time_last_itr;
  ros::Publisher map_publisher_,laser_publisher_,point2_publisher_,odom_publisher_,adjusted_odom_publisher_,fuser_odom_publisher_;
  nav_msgs::Odometry fuser_odom,adjusted_odom_msg;
  Eigen::Affine3d last_odom, this_odom,last_gt_pose;
  velodyne_rawdata::RawData velodyne_parser;

  bool use_tf_listener_;
  Eigen::Affine3d last_tf_frame_;
  perception_oru::MotionModel2d::Params motion_params;
  boost::mutex m;
public:
  // Constructor
  GraphMapFuserNode(ros::NodeHandle param_nh) : frame_nr_(0)
  {
    ///if we want to build map reading scans directly from bagfile


    ///topic to wait for point clouds, if available
    param_nh.param<std::string>("points_topic",points_topic,"");

    ///enable for LaserScan message input
    param_nh.param("laser_2d",matchLaser,true);


    ///range to cutoff sensor measurements
    param_nh.param<double>("max_range",max_range,30.0);
    //minimum range of sensor measurments
    param_nh.param("min_range",min_range,0.1);

    param_nh.param<string>("velodyne_config_file",velodyne_config_file,"");

    ///visualize in a local window
    param_nh.param("visualize",visualize,true);

    std::string marker_str;
    param_nh.param<std::string>("plot_marker",marker_str,"point");
    if(marker_str.compare("sphere")==0)
      plot_marker=plotmarker::sphere;
    else if(marker_str.compare("point")==0)
      plot_marker=plotmarker::point;
    else
      plot_marker=plotmarker::sphere;

    ///if using the HMT fuser, NDT maps are saved in this directory.
    ///a word of warning: if you run multiple times with the same directory,
    ///the old maps are loaded automatically
    param_nh.param<std::string>("map_directory",map_dir,"/maps/");

    param_nh.param<std::string>("map_type",map_type_name,"ndt_map");
    param_nh.param<std::string>("registration_type",reg_type_name,"default_reg");

    param_nh.param<std::string>("file_format_map",file_format_map,".map");

    ///initial pose of the vehicle with respect to the map
    param_nh.param("pose_init_x",pose_init_x,0.);
    param_nh.param("pose_init_y",pose_init_y,0.);
    param_nh.param("pose_init_z",pose_init_z,0.);
    param_nh.param("pose_init_r",pose_init_r,0.);
    param_nh.param("pose_init_p",pose_init_p,0.);
    param_nh.param("pose_init_t",pose_init_t,0.);

    ///pose of the sensor with respect to the vehicle odometry frame
    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
    param_nh.param("sensor_pose_t",sensor_pose_t,0.);
    param_nh.param("sensor_offset_t",sensor_offset_t_,0.);
    ///size of the map in x/y/z. if using HMT, this is the size of the central tile
    param_nh.param("size_x_meters",size_x,10.);
    param_nh.param("size_y_meters",size_y,10.);
    param_nh.param("size_z_meters",size_z,10.);

    param_nh.param<double>("motion_params_Cd", motion_params.Cd, 0.005);
    param_nh.param<double>("motion_params_Ct", motion_params.Ct, 0.01);
    param_nh.param<double>("motion_params_Dd", motion_params.Dd, 0.001);
    param_nh.param<double>("motion_params_Dt", motion_params.Dt, 0.01);
    param_nh.param<double>("motion_params_Td", motion_params.Td, 0.001);
    param_nh.param<double>("motion_params_Tt", motion_params.Tt, 0.005);

    bool do_soft_constraints;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
    param_nh.param("laser_variance_z",varz,0.1);

    ///if we want to create map based on GT pose
    param_nh.param("renderGTmap",gt_mapping,false);
    param_nh.param<std::string>("gt_topic",gt_topic,"groundtruth");
    ///if we want to get the initial pose of the vehicle relative to a different frame
    param_nh.param("initPoseFromGT",initPoseFromGT,false);
    //plot the map from the GT track if available

    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");

    param_nh.param<std::string>("odometry_adjusted",odometry_adjusted_topic,"odometry_adjusted");
    //get it from TF?
    param_nh.param("initPoseFromTF",initPoseFromTF,false);

    //the frame to initialize to

    param_nh.param<std::string>("world_frame",world_link_id,"/world");
    //our frame
    param_nh.param<std::string>("fuser_frame_id",fuser_base_link_id,"/fuser_base_link");
    param_nh.param<std::string>("laser_frame_id",laser_link_id,"/laser_link");

    param_nh.param<std::string>("state_base_link_id",state_base_link_id,"/state_base_link");

    ///use standard odometry messages for initialuess
    param_nh.param("useOdometry",useOdometry,true);

    param_nh.param<bool>("use_tf_listener", use_tf_listener_, false);
    param_nh.param<std::string>("odometry_frame_id", odometry_link_id, std::string("/odom_base_link"));

    initPoseSet = false;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
    fuser_odom.header.frame_id="/world";
    adjusted_odom_msg.header.frame_id="/world";
    laser_publisher_ =param_nh.advertise<sensor_msgs::LaserScan>("laserscan_in_fuser_frame",50);

    point2_publisher_ =param_nh.advertise<sensor_msgs::PointCloud2>("point2_fuser",15);
    fuser_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("fuser",50);
    adjusted_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("odom_gt_init",50);

    if(gt_mapping)
      use_tf_listener_= use_tf_listener_ && state_base_link_id != std::string("");// check if odometry topic exists
    else
      use_tf_listener_= use_tf_listener_ && odometry_link_id != std::string("");// check if odometry topic exists

    sensorPose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
        Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

    tf::poseEigenToTF(sensorPose_,tf_sensor_pose_);



    if(!matchLaser) {
      velodyne_parser.setParameters(min_range, max_range ,0 , 2*M_PI);
      velodyne_parser.setupOffline(velodyne_config_file,max_range,min_range);
      points2_sub_ = new message_filters::Subscriber<velodyne_msgs::VelodyneScan>(nh_,points_topic,2);
      if(useOdometry) {
        if(gt_mapping){
          if(!use_tf_listener_){
            std::cout<<"Using synchonized gt odometry and velodyne messages."<<std::endl;
            gt_fuser_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,gt_topic,10);
            sync_GTodom_ = new message_filters::Synchronizer< PointsGTOdomSync >(PointsGTOdomSync(SYNC_FRAMES), *points2_sub_, *gt_fuser_sub_);
            sync_GTodom_->registerCallback(boost::bind(&GraphMapFuserNode::GTLaserPointsOdomCallback, this, _1, _2));
          }
          else{
            std::cout<<"Use tf for GT odometry estimates"<<std::endl;
            points2OdomTfSub=nh_.subscribe <velodyne_msgs::VelodyneScan>(points_topic,10,&GraphMapFuserNode::GTLaserPointsOdomCallbackTF,this);
          }
        }
        else{
          if(!use_tf_listener_){
            std::cout<<"Using synchronized odometry and velodyne messages"<<std::endl;
            odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
            sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
            sync_po_->registerCallback(boost::bind(&GraphMapFuserNode::points2OdomCallback, this, _1, _2));
          }
          else{
            std::cout<<"Use TF for odometry estimates"<<std::endl;
            points2OdomTfSub=nh_.subscribe <velodyne_msgs::VelodyneScan>(points_topic,10,&GraphMapFuserNode::points2OdomCallbackTF,this);
          }
        }
      }
    }
    else
    {
      laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,points_topic,2);
      if(useOdometry) {
        std::cout<<"Uisng synchronized laser scanner and odometry call back"<<std::endl;
        odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
        sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
        sync_lo_->registerCallback(boost::bind(&GraphMapFuserNode::laserOdomCallback, this, _1, _2));
      }
      else{
        ((void)0); //Do nothing, seriously consider using a laser only callback (no odometry sync)
      }
    }
    if(initPoseFromGT) {
      std::cout<<"Init pose using GT data from topic ="<<gt_topic<<std::endl;
      gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&GraphMapFuserNode::gt_callback, this);
    }
    else if(!initPoseFromGT){
      pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
          Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
      initPoseSet=true;
      std::cout<<"node: initial pose =\n"<<pose_.translation()<<std::endl;
    }

    save_map_ = param_nh.advertiseService("save_map", &GraphMapFuserNode::save_map_callback, this);
    NDTCell::setParameters(0.1, 8*M_PI/18.,1000, 3, false, false);

    if(points_topic=="")
      cerr<<"No topic specified"<<std::endl;
    else{

      std::cout<<"init done... waiting for data on topic\""<<points_topic<<"\""<<", odometry at: "<<odometry_topic<<std::endl;
    }
  }

  void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion) {

    if(!initPoseSet)
      return;

    frame_nr_++;
    std::cout<<"frame nr="<<frame_nr_<<", cloud size=<<"<<cloud.size()<<std::endl;
    if(fuser_==NULL){
      fuser_=new GraphMapFuser(map_type_name,reg_type_name,pose_*sensorPose_,sensorPose_);
      std::cout<<"----------------------------FUSER------------------------"<<std::endl;
      std::cout<<fuser_->ToString()<<std::endl;
      fuser_->Visualize(visualize,plot_marker);
      std::cout<<"---------------------------------------------------------"<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ> registered_cloud=cloud;
    ros::Time tplot=ros::Time::now();
    Eigen::MatrixXd cov(6,6);
    cov=Eigen::MatrixXd::Identity(6,6);
    m.lock();
    bool uppdated=fuser_->ProcessFrame<pcl::PointXYZ>(cloud,pose_,Tmotion,cov);
    m.unlock();
    fuser_->PlotMapType();
    tf::Transform Transform;
    tf::transformEigenToTF(pose_,Transform);
    tf_.sendTransform(tf::StampedTransform(Transform, tplot, world_link_id, fuser_base_link_id));
    tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, tplot, fuser_base_link_id, laser_link_id));
    fuser_odom.header.stamp=tplot;
    plotPointcloud2(registered_cloud,tplot);
    tf::poseEigenToMsg( pose_,fuser_odom.pose.pose);
    fuser_odom_publisher_.publish(fuser_odom);
  }

  //bool save_map_callback(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res )
  bool ResetInvalidMotion(Eigen::Affine3d &Tmotion){
    if(Tmotion.translation().norm() > MAX_TRANSLATION_DELTA) {
      std::cerr<<"Ignoring Odometry (max transl)!\n";
      std::cerr<<Tmotion.translation().transpose()<<std::endl;
      Tmotion.setIdentity();
      return true;
    }
    else if(Tmotion.rotation().eulerAngles(0,1,2)(2) > MAX_ROTATION_DELTA) {
      std::cerr<<"Ignoring Odometry (max rot)!\n";
      std::cerr<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
      Tmotion.setIdentity();
      return true;
    }
    else return false;
  }
  bool exists_test0 (const std::string& name);
  bool save_map_callback(std_srvs::Empty::Request  &req,
                         std_srvs::Empty::Response &res ) {
    char path[1000];
    string time=ndt_generic::currentDateTimeString();

    if(fuser_!=NULL){

      m.lock();
      if(file_format_map.compare(".JFF")==0)
        fuser_->SaveCurrentNodeAsJFF(map_dir+"/"+"ndt_map.JFF"); //snprintf(path,999,"%s/ndt_map_.JFF",map_dir.c_str(),)
      else
        fuser_->SaveGraphMap(map_dir+"/"+"ndt_map.MAP");

      m.unlock();
      return true;
    }
    else
      ROS_INFO("No data to save");
    return false;
  }

  inline bool getAffine3dTransformFromTF(const ros::Time &time,const std::string &link_id,Eigen::Affine3d& ret,const ros::Duration &wait) {
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    tf_listener_.waitForTransform(world_link_id, link_id,  time ,wait);
    try{
      tf_listener_.lookupTransform(world_link_id, link_id, time, transform);
      tf::poseTFToEigen(transform, ret);
      std::cout<<"found "<<ret.translation().transpose()<<std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
    return true;
  }

  // Callback
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
    std::cout<<"laser callback"<<std::endl;
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
    projector_.projectLaser(*msg_in, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZ pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_range) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    T.setIdentity();
    std::cout<<"node: laser call back, process frame"<<std::endl;
    this->processFrame(pcl_cloud,T);
  }

  // Callback
  void laserOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
                         const nav_msgs::Odometry::ConstPtr& odo_in)
  {
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_unfiltered;
    Eigen::Affine3d Tm;

    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);
    if (frame_nr_  <= 1){
      Tm.setIdentity();
    }
    else
      Tm = last_odom.inverse()*this_odom;

    last_odom = this_odom;
    projector_.projectLaser(*msg_in, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);
    sensor_msgs::LaserScan msg_out=*msg_in;
    msg_out.header.stamp=ros::Time::now();
    msg_out.header.frame_id="/fuser_laser_link";
    laser_publisher_.publish(msg_out);
    pcl::PointXYZ pt;

    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_range) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    pcl_cloud.height=1;
    pcl_cloud.width=pcl_cloud.size();
    plotPointcloud2(pcl_cloud);
    this->processFrame(pcl_cloud,Tm);

    if(frame_nr_%10==0)
      fuser_->PlotMapType();
  }

  void plotPointcloud2(pcl::PointCloud<pcl::PointXYZ> & cloud,ros::Time time = ros::Time::now()){
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(cloud,msg_out);
    msg_out.header.frame_id=laser_link_id;
    msg_out.header.stamp=time;
    point2_publisher_.publish(msg_out);
  }
  void points2OdomCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg_in,
                           const nav_msgs::Odometry::ConstPtr& odo_in)//callback is used in conjunction with odometry time filter.
  {
    ros::Time tstart=ros::Time::now();

    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);

    Eigen::Affine3d Tm;
    pcl::PointCloud<pcl::PointXYZ> cloud;


    if (frame_nr_ == 0)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom = this_odom;
    //pcl::fromROSMsg (*msg_in, cloud);
    ros::Time t1=ros::Time::now();
    ndt_generic::UnwarpCloudSimple<pcl::PointXYZ>(velodyne_parser,msg_in,cloud);
    ros::Time t2=ros::Time::now();
    std::cout<<"Duration unwarp="<<t2-t1<<std::endl;
    this->processFrame(cloud,Tm);
    ros::Time tend=ros::Time::now();
    std::cout<<"Total execution time= "<<tend-tstart<<std::endl;
  }
  void points2OdomCallbackTF(const velodyne_msgs::VelodyneScan::ConstPtr& msg_in){//this callback is used to look up tf transformation for scan data
    Eigen::Affine3d Tm;
    static bool last_odom_found=false;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    //pcl::fromROSMsg (*msg_in, cloud);
    ndt_generic::UnwarpCloudSimple<pcl::PointXYZ>(velodyne_parser,msg_in,cloud);

    bool found_odom= getAffine3dTransformFromTF((msg_in->header.stamp-ros::Duration(sensor_offset_t_)),odometry_link_id,this_odom,ros::Duration(0.1));
    if (frame_nr_ =0 || !found_odom||!last_odom_found)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom_found=found_odom;

    last_odom = this_odom;
    this->processFrame(cloud,Tm);
    fuser_->PlotMapType();

  }

  void GTLaserPointsOdomCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg_in,
                                 const nav_msgs::Odometry::ConstPtr& odo_in)//this callback is used for GT based mapping
  {
    Eigen::Affine3d Tmotion;
    if(frame_nr_==0){
      Tmotion=Eigen::Affine3d::Identity();
    }
    Eigen::Affine3d GT_pose;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf::poseMsgToEigen(odo_in->pose.pose,pose_);
    //pcl::fromROSMsg (*msg_in, cloud);
    ndt_generic::UnwarpCloudSimple(velodyne_parser,msg_in,cloud);
    ros::Time t_stamp=ros::Time::now();//msg_in->header.stamp;
    tf::Transform gt_base;
    tf::poseMsgToTF(odo_in->pose.pose,gt_base);
    tf_.sendTransform(tf::StampedTransform(gt_base, t_stamp, world_link_id, std::string("online_")+state_base_link_id));
    tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, t_stamp, std::string("online_")+state_base_link_id, laser_link_id));
    plotPointcloud2(cloud);
    m.lock();
    fuser_->ProcessFrame(cloud,pose_,Tmotion);
    fuser_->PlotMapType();
    m.unlock();
  }
  void GTLaserPointsOdomCallbackTF(const velodyne_msgs::VelodyneScan::ConstPtr& msg_in)//this callback is used for GT based mapping with TF lookup
  {
    Eigen::Affine3d tmp_pose;
    Eigen::Affine3d Tmotion=Eigen::Affine3d::Identity();
    bool found_odom= getAffine3dTransformFromTF((msg_in->header.stamp-ros::Duration(sensor_offset_t_)),state_base_link_id,tmp_pose,ros::Duration(0.1));

    if(found_odom){
      pose_=tmp_pose;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      //pcl::fromROSMsg (*msg_in, cloud);
      ndt_generic::UnwarpCloudSimple<pcl::PointXYZ>(velodyne_parser,msg_in,cloud);
      ros::Time t_stamp=ros::Time::now();//msg_in->header.stamp;
      tf::Transform tf_gt_base;
      tf::poseEigenToTF(pose_,tf_gt_base);
      tf_.sendTransform(tf::StampedTransform(tf_gt_base, t_stamp, world_link_id, std::string("online_")+state_base_link_id));
      tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, t_stamp, std::string("online_")+state_base_link_id, laser_link_id));
      plotPointcloud2(cloud,t_stamp);
      fuser_->ProcessFrame(cloud,pose_,Tmotion);
      fuser_->PlotMapType();
      m.unlock();
    }

  }
  // Callback
  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
    if(initPoseFromGT && !initPoseSet) {
      Eigen::Affine3d gt_pose;
      tf::poseMsgToEigen(msg_in->pose.pose,gt_pose);
      pose_ = gt_pose;
      std::cout<<"node: initial pose =\n"<<pose_.translation()<<std::endl;
      initPoseSet = true;
    }
  }

public:
  // map publishing function
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_map_fuser_node");
  ros::NodeHandle param("~");
  GraphMapFuserNode t(param);
  ros::spin();

  return 0;
}



