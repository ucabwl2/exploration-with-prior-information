//perception_oru
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "graph_map/graph_map_navigator.h"
#include "graph_map/graphfactory.h"

#include "ndt_localization/particle_filter.hpp"
#include "ndt_map/ndt_map.h"
#include "ndt_map/ndt_conversions.h"
//pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//ros
#include "geometry_msgs/PoseArray.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/rate.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>
#include "eigen_conversions/eigen_msg.h"
//std
#include <Eigen/Dense>
#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <chrono>
#include "graph_localization/mcl_ndt/mcl_ndt.h"
#include "graph_localization/localization_factory.h"
#include "ndt_generic/motionmodels.h"
#include "ndt_generic/motion_model_2d.h"
using namespace perception_oru;
using namespace graph_map;
using namespace graph_localization;
class localisation_node {
  ros::NodeHandle nh;
  //Map parameters
  std::string map_path;
  GraphMapNavigatorPtr graph_map_;
  LocalisationTypePtr localisation_type_ptr_;
  //MCL
  bool visualize;
  std::string initType;
  ros::Publisher mclPosePub;
  //laser input
  std::string points_topic, gt_topic;   //std::string laserTopicName;

  ros::Publisher cloud_pub;
  ros::Subscriber initPoseSub;
  ros::Subscriber gtPoseSub;
  ros::Subscriber PCSub;

  Eigen::Affine3d Tsens;
  std::string rootTF;
  std::string odomTF;
  std::string baseTF;
  std::string mclTF;
  string localisation_type_name="";
  string dataset="";

  Eigen::Affine3d initial_pose;
  Eigen::Affine3d tOld;
  bool firstLoad;
  geometry_msgs::PoseArray parMsg;
  double minx;
  double miny;


  double initVar;
  double var_x;
  double var_y;
  double var_th;
  double r_var_x;
  double r_var_y;
  double r_var_th;
  int tres;
  int counter;
  int counterLimit;

  std::ofstream res;
  std::string resF_name;
  bool initialized;
  double time_0;

  velodyne_rawdata::RawData dataParser;
  double min_range;
  double max_range;

  double v_size_x;
  double v_size_y;
  double v_size_z;
  double fraction;

  bool laser_2d;
  bool beVelodyne;
  bool bePC;
  bool init_pose_gt;
  bool initial_pose_set;
  void Pose2DToTF(Eigen::Vector3d mean, ros::Time ts, Eigen::Affine3d Todometry){
    static tf::TransformBroadcaster br, br_mapOdom;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, mean[2]);
    transform.setOrigin( tf::Vector3(mean[0], mean[1], 0.0) );
    transform.setRotation( q );
    br.sendTransform(tf::StampedTransform(transform, ts, rootTF, mclTF));

    // // br.sendTransform(tf::StampedTransform(transform, ts, rootTF, "/mcl_pose"));
    // ///Compute TF between map and odometry frame
    // Eigen::Affine3d Tmcl = getAsAffine(mean[0], mean[1], mean[2]);
    // Eigen::Affine3d Tmap_odo = Tmcl * Todometry.inverse();
    // tf::Transform tf_map_odo;
    // tf_map_odo.setOrigin( tf::Vector3(Tmap_odo.translation() (0), Tmap_odo.translation() (1), 0.0) );
    // tf::Quaternion q_map_odo;
    // q_map_odo.setRPY( 0, 0, Tmap_odo.rotation().eulerAngles(0, 1, 2) (2) );
    // tf_map_odo.setRotation( q_map_odo );
    // // /// broadcast TF
    //br_mapOdom.sendTransform(tf::StampedTransform(tf_map_odo, ts + ros::Duration(0.3), rootTF, odomTF));
  }

  int LoadMap(){
    LoadGraphMap(map_path,graph_map_);
    if(graph_map_==NULL){
      std::cerr<<"ERROR LOADING NDT MAP FROM FILE"<<std::endl;
      exit(0);
    }
  }
  void Initialize(const Eigen::Affine3d &pose_init){
    geometry_msgs::Pose pose_init_geom;
    tf::poseEigenToMsg(pose_init,pose_init_geom);
    Initialize(pose_init_geom,ros::Time::now());
  }
  void initialposeCallback(geometry_msgs::PoseWithCovarianceStamped input_init){
    Initialize(input_init.pose.pose,input_init.header.stamp);
    initialized = true;
    firstLoad = true;
  }
  void GTCallback(const nav_msgs::Odometry::ConstPtr& msg_in){
    if(!initialized){
      cout<<"initialize pose calllback"<<endl;
      Initialize(msg_in->pose.pose,msg_in ->header.stamp);
      initialized = true;
      firstLoad = true;
      gtPoseSub.shutdown();
    }
  }
  void Initialize(const geometry_msgs::Pose &pose_init,const ros::Time &t_init){

    Eigen::Affine3d pose_init_eig;
    tf::poseMsgToEigen(pose_init,pose_init_eig);
    cout<<"Initial position set to"<<pose_init_eig.translation().transpose()<<endl;
    Vector6d var;
    var<<0.5, 0.5, 0.0, 0.0, 0.0, 0.2;
    localisation_type_ptr_->InitializeLocalization(pose_init_eig,var);
    sleep(1);
    initialized=true;
  }
  void VeloCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg){
    cout<<"velodyne callback"<<endl;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(size_t next = 0; next < msg->packets.size(); ++next){
      velodyne_rawdata::VPointCloud pnts;
      dataParser.unpack(msg->packets[next], pnts);
      for(size_t i = 0; i < pnts.size(); i++){
        pcl::PointXYZ p;
        p.x = pnts.points[i].x;
        p.y = pnts.points[i].y;
        p.z = pnts.points[i].z;
        cloud.push_back(p);
      }
      pnts.clear();
    }
    this->processFrame(cloud,msg->header.stamp);
  }
  void PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg (*msg, pcl_cloud);
    this->processFrame(pcl_cloud,msg->header.stamp);
  }

  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
    //      message_m.lock();
    projector_.projectLaser(*msg, cloud);
    //message_m.unlock();
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZ pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_range) {
        pt.z += 0.1*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    this->processFrame(pcl_cloud,msg->header.stamp);
  }

  void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time ts){
    static tf::TransformListener tf_listener;

    pcl::PointCloud<pcl::PointXYZ> cloud_localized;
    cloud_localized=cloud;
    if(!initialized && !initial_pose_set)
      return;
    else if(!initialized && initial_pose_set){
      geometry_msgs::Pose pose_init_geom;
      tf::poseEigenToMsg(initial_pose,pose_init_geom);
      Initialize(pose_init_geom,ts);
    }


    tf::StampedTransform transform;
    double x, y, yaw;

    tf_listener.waitForTransform(odomTF, baseTF, ts, ros::Duration(0.1));
    try{
      tf_listener.lookupTransform(odomTF, baseTF,/*ros::Time(0)*/ts, transform);
      yaw = tf::getYaw(transform.getRotation());
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
    }
    Eigen::Affine3d T;
    tf::poseTFToEigen(transform,T);
    //Eigen::Affine3d T = getAsAffine(x, y, yaw);
    if(firstLoad){
      tOld = T;
      firstLoad = false;
    }
    Eigen::Affine3d Tmotion = tOld.inverse() * T;

    tOld = T;
    transformPointCloudInPlace(Tsens,cloud);
    localisation_type_ptr_->UpdateAndPredict(cloud,Tmotion);
    Eigen::Affine3d pose_=localisation_type_ptr_->GetPose();

    nav_msgs::Odometry mean_pose_odom;
    mean_pose_odom.header.frame_id="world";
    tf::poseEigenToMsg(pose_,mean_pose_odom.pose.pose);
    mean_pose_odom.header.stamp = ts;
    mclPosePub.publish(mean_pose_odom);
    if(visualize){
      transformPointCloudInPlace(pose_, cloud_localized);
      cloud_localized.header.frame_id="world";
      pcl_conversions::toPCL(ts,cloud_localized.header.stamp);
      cloud_pub.publish(cloud_localized);
    }
    cloud.clear();
    cloud_localized.clear();
  }

public:
  localisation_node(ros::NodeHandle param){
    param.param<std::string>("map_file", map_path, "");
    param.param<bool>("visualize", visualize, true);

    param.param<std::string>("points_topic", points_topic, "/velodyne_packets");
    param.param<std::string>("gt_topic", gt_topic, "");
    param.param<bool>("initPoseFromGT", init_pose_gt, true);
    param.param<bool>("Velodyne", beVelodyne, false);
    param.param<bool>("PointCloud", bePC, false);
    param.param<bool>("laser_2d", laser_2d, false);

    param.param<std::string>("root_tf", rootTF, "/map");
    param.param<std::string>("odom_tf", odomTF, "/odom");
    param.param<std::string>("base_tf", baseTF, "/base_link");
    param.param<std::string>("mcl_tf", mclTF, "/mcl");

    Eigen::Vector3d sensor_offset_pos, sensor_offset_euler;
    param.param("sensor_pose_x",sensor_offset_pos(0),0.);
    param.param("sensor_pose_y",sensor_offset_pos(1),0.);
    param.param("sensor_pose_z",sensor_offset_pos(2),0.);
    param.param("sensor_pose_r",sensor_offset_euler(0),0.);
    param.param("sensor_pose_p",sensor_offset_euler(1),0.);
    param.param("sensor_pose_t",sensor_offset_euler(2),0.);
    Tsens = ndt_generic::vectorsToAffine3d(sensor_offset_pos,sensor_offset_euler);

    Eigen::Vector3d init_pos = Eigen::Vector3d::Identity();
    Eigen::Vector3d init_euler = Eigen::Vector3d::Identity();
    param.param<double>("pose_init_x", init_pos(0), 0.0);
    param.param<double>("pose_init_y", init_pos(1), 0.0);
    param.param<double>("pose_init_tLas", init_euler(2), 0.0);
    param.param<double>("init_var", initVar, 0.5);

    param.param<double>("var_x", var_x, 0.07);
    param.param<double>("var_y", var_y, 0.07);
    param.param<double>("var_th", var_th, 0.035);

    param.param<double>("r_var_x", r_var_x, 1.0);
    param.param<double>("r_var_y", r_var_y, 1.0);
    param.param<double>("r_var_th", r_var_th, 0.001);
    param.param<std::string>("dataset", dataset, "oru-lab");

    param.param("min_range", min_range, 1.5);
    param.param("max_range", max_range, 100.0);

    param.param<std::string>("localisation_type_name", localisation_type_name, "mcl_ndt");
    int retu = dataParser.setup(param);
    dataParser.setParameters(min_range, max_range, 0, 2 * 3.1415);

    LoadMap();
    LocalisationParamPtr  loc_ptr=LocalisationFactory::CreateLocalisationParam(localisation_type_name);

    loc_ptr->GetParamFromRos();
    loc_ptr->sensor_pose=Tsens;
    loc_ptr->graph_map_=graph_map_;

    if(MCLNDTParamPtr parPtr=boost::dynamic_pointer_cast<MCLNDTParam>(loc_ptr )){
      cout<<"Read motion model for MCL"<<endl;
      GetMotionModel(dataset,parPtr->motion_model);
    }

    cout<<"----------------Localisation parameters------------------\n"<<loc_ptr->ToString()<<endl;
    localisation_type_ptr_=LocalisationFactory::CreateLocalisationType(loc_ptr);
    cout<<"---------------------------------------------------------"<<endl;

    firstLoad = true;
    initialized = false;
    res.open(resF_name);


    if(beVelodyne){
      PCSub = nh.subscribe(points_topic, 1, &localisation_node::VeloCallback, this);
      cout<<"Listen to sensor_msgs::PointCloud2 at topic \""<<points_topic<<"\""<<endl;
    }
    else if(bePC){
      PCSub = nh.subscribe(points_topic, 1, &localisation_node::PCCallback, this);
      cout<<"Listen to sensor_msgs::velodyne_msgs::VelodyneScan at topic \""<<points_topic<<"\""<<endl;
    }
    else if(laser_2d){
      PCSub = nh.subscribe(points_topic, 1, &localisation_node::LaserCallback, this);
      cout<<"Listen to 2d-laser at topic \""<<points_topic<<"\""<<endl;
    }
    else
      std::cerr<<"No lidar Type sepected for topic \""<<points_topic<<"\""<<std::endl;

    initPoseSub = nh.subscribe("/initialpose", 1000, &localisation_node::initialposeCallback, this);

    if(init_pose_gt){
      cout<<"Listen to GT at topic :"<<gt_topic<<endl;
      gtPoseSub= nh.subscribe<nav_msgs::Odometry>(gt_topic, 10, &localisation_node::GTCallback, this);
    }
    else{
      initial_pose = ndt_generic::vectorsToAffine3d(init_pos,init_euler);
      cout<<"Initial pose set to :"<<initial_pose.translation().transpose()<<endl;
      initial_pose_set=true;
    }


    mclPosePub=nh.advertise<nav_msgs::Odometry>("ndt_mcl_pose", 20);

    cloud_pub=nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud_localized", 20);
    ros::spin();
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "ndt_mcl");
  ros::NodeHandle parameters("~");
  localisation_node pf(parameters);
}


