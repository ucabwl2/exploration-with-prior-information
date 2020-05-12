#include <ndt_fuser/ndt_fuser_hmt.h>
#include <ndt_offline/VelodyneBagReader.h>
#include <ndt_generic/eigen_utils.h>
#include <ndt_generic/pcl_utils.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>
#include <boost/program_options.hpp>
#include "graph_map/graph_map_fuser.h"
#include "graph_map/ndt/ndt_map_param.h"
#include "graph_map/ndt/ndtd2d_reg_type.h"
#include "graph_map/ndt/ndt_map_type.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ros/publisher.h"
#include "tf/transform_broadcaster.h"
#include "ndt_generic/eigen_utils.h"
#include "ndt_generic/io.h"
#include "ndt_offline/pointcloudbagreader.h"
#include "ndt_offline/readbagfilegeneric.h"
#include <unistd.h>
#include "tf_conversions/tf_eigen.h"
#include "ndt_generic/io.h"
#include "ndt_generic/sensors_utils.h"
#include "ndt_offline/readpointcloud.h"
#include "ndt_generic/motionmodels.h"
#include "ndt_generic/motion_model_3d.h"

using namespace perception_oru;
using namespace graph_map;
namespace po = boost::program_options;
using namespace std;

Eigen::Vector3d transl;
Eigen::Vector3d euler;
std::string map_dir_name="";
std::string output_dir_name="";
std::string base_name="";
std::string dataset="";
std::string bag_reader_type="";
//map parameters
int itrs=0;
int attempts=1;
int nb_neighbours=0;
int nb_scan_msgs=0;
bool use_odometry_source=false;
double lambda=1.0;
bool visualize=true;
bool use_multires=false;
bool beHMT=false;
bool filter_cloud=false;
bool step_control=false;
bool check_consistency=true;
bool registration2d=true;
bool use_submap=true;
double min_keyframe_dist=0.5;
double min_keyframe_dist_rot_deg=15;
bool use_keyframe=true;
bool alive=false;
bool save_map=true;
bool gt_mapping=false;
bool disable_reg=false, do_soft_constraints=false;
bool pcl_reader=true;
bool init_pose_gt_frame=false;
perception_oru::MotionModel2d::Params motion_params;
std::string base_link_id="", gt_base_link_id="", tf_world_frame="";
std::string velodyne_config_file="";
std::string lidar_topic="";
std::string velodyne_frame_id="";
std::string map_type_name="",registration_type_name="";
std::string tf_topic="";
tf::Transform Tsensor_offset_tf;
std::string map_switching_method="";
ndt_generic::Vector6d init;
ros::NodeHandle *n_=NULL;
RegParamPtr regParPtr=NULL;
MapParamPtr mapParPtr=NULL;
GraphMapNavigatorParamPtr graphParPtr=NULL;
MotionModel3d mot_model;

double sensor_time_offset=0;
double map_size_xy=0;
double map_size_z=0;
double resolution_local_factor=0;
double max_range=0, min_range=0;
double maxRotationNorm=0;
double compound_radius=0;
double interchange_radius=0;
double distance_alpha=1.0;
double maxTranslationNorm=0;
double rotationRegistrationDelta=0;
double sensorRange=30;
double translationRegistrationDelta=0;
double resolution=0;
double hori_min=0, hori_max=0;
double z_min=0, z_max=0;
unsigned int skip_frame=20;
ros::Publisher *gt_pub,*fuser_pub,*cloud_pub,*odom_pub;
nav_msgs::Odometry gt_pose_msg,fuser_pose_msg,odom_pose_msg;
bool use_pointtype_xyzir;
int min_nb_points_for_gaussian;
bool keep_min_nb_points;
bool min_nb_points_set_uniform;
int nb_measurements=1;
int max_nb_iters=30;
bool generate_eval_files = false;
bool use_only_static_scans = false;
bool save_used_merged_clouds = false;
bool save_graph_cloud=false;
bool maptype_cloud=false;
bool use_gt_data=false;
int nb_frames = 0;
int counter = 0;
ndt_generic::Vector6d sigma;
ndt_offline::OdometryType odom_type;
Eigen::Affine3d Todom_base_prev,Tgt_base_prev, Tgt_base, Todom_base, odom_pose, fuser_pose, Tsensor_offset;
Eigen::MatrixXd predCov;
GraphMapFuser *fuser_=NULL;
template<class T> std::string toString (const T& x)
{
  std::ostringstream o;

  if (!(o << x))
    throw std::runtime_error ("::toString()");

  return o.str ();
}

std::string transformToEvalString(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Quaternion<double> tmp(T.rotation());
  stream << T.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << std::endl;
  return stream.str();
}




bool LocateRosBagFilePaths(const std::string &folder_name,std::vector<std::string> &scanfiles){
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (folder_name.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      if(ent->d_name[0] == '.') continue;
      char tmpcname[400];
      snprintf(tmpcname,399,"%s/%s",folder_name.c_str(),ent->d_name);
      std::string tmpfname = tmpcname;
      scanfiles.push_back(tmpfname);
    }
    closedir (dir);
  } else {
    std::cerr<<"Could not parse dir name\n";
    return false;
  }
  sort(scanfiles.begin(),scanfiles.end());
  {
    std::cout << "files to be loaded : " << std::endl;
    for (size_t i = 0; i < scanfiles.size(); i++) {
      std::cout << " " << scanfiles[i] << std::flush;
    }
    std::cout << std::endl;
  }
  return true;
}
bool ReadAllParameters(po::options_description &desc,int &argc, char ***argv){

  double Tx,Ty,Tz;
  double nickes_arg;
  // First of all, make sure to advertise all program options
  desc.add_options()
      ("help", "produce help message")
      ("map-type-name", po::value<string>(&map_type_name)->default_value(std::string("ndt_map")), "type of map to use e.g. ndt_map or ndt_dl_map (default it default)")
      ("registration-type-name", po::value<string>(&registration_type_name)->default_value(std::string("ndt_d2d_reg")), "type of map to use e.g. ndt_d2d_reg or ndt_dl_reg (default it default)")
      ("visualize", "visualize the output")
      ("disable-odometry", "dont use odometry as initial guess")
      ("disable-mapping", "build maps from cloud data")
      ("attempts", po::value<int>(&attempts)->default_value(1), "Total retries of localisation, can be used to generate multiple files")
      ("step-control", "use step control in the optimization (default=false)")
      ("base-name", po::value<string>(&base_name)->default_value("off"), "prefix for all generated files")
      ("reader-type", po::value<string>(&bag_reader_type)->default_value("velodyne_reader"), "e.g. velodyne_reader or pcl_reader")
      ("data-set", po::value<string>(&dataset)->default_value(std::string("")), "choose which dataset that is currently used, this option will assist with assigning the sensor pose")
      ("dir-name", po::value<string>(&map_dir_name), "where to look for ros bags")
      ("map-switching-method", po::value<string>(&map_switching_method)->default_value("node_position"), "where to look for ros bags")
      ("output-dir-name", po::value<string>(&output_dir_name)->default_value("/home/daniel/.ros/maps"), "where to save the pieces of the map (default it ./map)")
      ("map-size-xy", po::value<double>(&map_size_xy)->default_value(83.), "size of submaps")
      ("map-size-z", po::value<double>(&map_size_z)->default_value(6.0), "size of submaps")
      ("itrs", po::value<int>(&itrs)->default_value(30), "number of iteration in the registration")
      ("fuse-incomplete", "fuse in registration estimate even if iterations ran out. may be useful in combination with low itr numbers")
      ("filter-cloud", "cutoff part of the field of view") //replaces filter-fov (its made more general
      ("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
      ("hori-min", po::value<double>(&hori_min)->default_value(-2*M_PI), "the minimum field of view angle horizontal")
      ("z-min", po::value<double>(&z_min)->default_value(DBL_MIN), "minimum height in lidar frame")
      ("z-max", po::value<double>(&z_max)->default_value(DBL_MAX), "maximum_height in lidar frame")
      ("Dd", po::value<double>(&motion_params.Dd)->default_value(1.), "forward uncertainty on distance traveled")
      ("Dt", po::value<double>(&motion_params.Dt)->default_value(1.), "forward uncertainty on rotation")
      ("Cd", po::value<double>(&motion_params.Cd)->default_value(1.), "side uncertainty on distance traveled")
      ("Ct", po::value<double>(&motion_params.Ct)->default_value(1.), "side uncertainty on rotation")
      ("Td", po::value<double>(&motion_params.Td)->default_value(1.), "rotation uncertainty on distance traveled")
      ("Tt", po::value<double>(&motion_params.Tt)->default_value(1.), "rotation uncertainty on rotation")
      ("tf-base-link", po::value<std::string>(&base_link_id)->default_value(std::string("")), "tf_base_link")
      ("tf-gt-link", po::value<std::string>(&gt_base_link_id)->default_value(std::string("")), "tf ground truth link")
      ("velodyne-config-file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("../config/velo32.yaml")), "configuration file for the scanner")
      ("tf-world-frame", po::value<std::string>(&tf_world_frame)->default_value(std::string("/world")), "tf world frame")
      ("lidar-topic", po::value<std::string>(&lidar_topic)->default_value(std::string("/velodyne_packets")), "velodyne packets topic used")
      ("velodyne-frame-id", po::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")), "frame_id of the laser sensor")
      ("alive", "keep the mapper/visualization running even though it is completed (e.g. to take screen shots etc.")
      ("nb_neighbours", po::value<int>(&nb_neighbours)->default_value(2), "number of neighbours used in the registration")
      ("min-range", po::value<double>(&min_range)->default_value(0.6), "minimum range used from scanner")
      ("max-range", po::value<double>(&max_range)->default_value(130), "minimum range used from scanner")
      ("save-map", "saves the graph map at the end of execution")
      ("nb_scan_msgs", po::value<int>(&nb_scan_msgs)->default_value(1), "number of scan messages that should be loaded at once from the bag")
      ("disable-keyframe-update", "use every scan to update map rather than update map upon distance traveled")
      ("keyframe-min-distance", po::value<double>(&min_keyframe_dist)->default_value(0.5), "minimum range used from scanner")
      ("keyframe-min-rot-deg", po::value<double>(&min_keyframe_dist_rot_deg)->default_value(15), "minimum range used from scanner")
      ("gt-mapping", "disable registration and use ground truth as input to mapping")
      ("tf-topic", po::value<std::string>(&tf_topic)->default_value(std::string("/tf")), "tf topic to listen to")
      ("Tx", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("Ty", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("Tz", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("Rex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("Rey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("Rez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("lambda-sc", po::value<double>(&lambda)->default_value(100), "lambda for sc")
      ("sx", po::value<double>(&sigma[0])->default_value(0.1), "Lambda sigma - x")
      ("sy", po::value<double>(&sigma[1])->default_value(0.1), "Lambda sigma - y")
      ("sz", po::value<double>(&sigma[2])->default_value(0.1), "Lambda sigma - z")
      ("sex", po::value<double>(&sigma[3])->default_value(0.1), "Lambda sigma - ex")
      ("sey", po::value<double>(&sigma[4])->default_value(0.1), "Lambda sigma - ey")
      ("sez", po::value<double>(&sigma[5])->default_value(0.1), "Lambda sigma - ez")
      ("init-x", po::value<double>(&init[0])->default_value(0.00), "init-x")
      ("init-y", po::value<double>(&init[1])->default_value(0.0), "init-y")
      ("init-z", po::value<double>(&init[2])->default_value(0.0), "init-z")
      ("init-ex", po::value<double>(&init[3])->default_value(0.0), "init-ex")
      ("init-ey", po::value<double>(&init[4])->default_value(0.0), "init-ey")
      ("init-ez", po::value<double>(&init[5])->default_value(0.0), "init-ez")
      ("skip-frame", po::value<unsigned int>(&skip_frame)->default_value(20), "sframes to skip before plot map etc.")
      ("sensor-time-offset", po::value<double>(&sensor_time_offset)->default_value(0.), "timeoffset of the scanner data")
      ("registration3d","registration3d") //Do not limit registration to the plane (x,y,theta)
      ("disable-registration", "Disable Registration")
      ("soft-constraints", "Use soft constraints in the registration")
      ("init-pose-gt-frame","Align first map node frame with ground truth frame")
      ("check-consistency", "if consistency should be checked after registration")("multi-res", "multi resolution registration")
      ("consistency-max-rot",po::value<double>(&maxRotationNorm)->default_value(0.8),"maxRotationNorm")
      ("consistency-max-dist",po::value<double>(&maxTranslationNorm)->default_value(0.4),"maxTranslationNorm")
      ("alpha", po::value<double>(&distance_alpha)->default_value(0.0), "alpha in the distance formula") //d=alpha*t+(1-alpha)*k*rot;
      ("translationRegistrationDelta",po::value<double>(&translationRegistrationDelta)->default_value(1.5),"sensorRange")
      ("resolution", po::value<double>(&resolution)->default_value(0.4), "resolution of the map")
      ("resolution-local-factor", po::value<double>(&resolution_local_factor)->default_value(1.), "resolution factor of the local map used in the match and fusing step")
      ("disable-submaps", "Adopt the sub-mapping technique which represent the global map as a set of local submaps")
      ("compound-radius", po::value<double>(&compound_radius)->default_value(10.0), "Requires sub-mapping enabled, When creating new sub-lamps, information from previous map is transfered to the new map. The following radius is used to select the map objects to transfer")
      ("interchange-radius", po::value<double>(&interchange_radius)->default_value(10.0), "This radius is used to trigger creation or selection of which submap to use")
      ("use-pointtype-xyzir", "If the points to be processed should contain ring and intensity information (velodyne_pointcloud::PointXYZIR)")
      ("min-nb-points-for-gaussian", po::value<int>(&min_nb_points_for_gaussian)->default_value(6), "minimum number of points per cell to compute a gaussian")
      ("keep-min-nb-points", "If the number of points stored in a NDTCell should be cleared if the number is less than min_nb_points_for_gaussian")
      ("min-nb-points-set-uniform", "If the number of points of one cell is less than min_nb_points_for_gaussian, set the distribution to a uniform one (cov = Identity)")
      ("nb-measurements", po::value<int>(&nb_measurements)->default_value(1), "number of scans collecte at each read iteration")
      ("max-nb-iters", po::value<int>(&max_nb_iters)->default_value(30), "max number of iterations used in the registration")
      ("generate-eval-files", "generate evaluation files at each used scan pose, storing the affine poses")
      ("save-used-merged_clouds", "incase of using static scans, multiple scans is used for each update, this will save the merged clouds")
      ("store-points","store pointclouds used to create the graph_map")
      ("save-graph-cloud", "Save a pointcloud corresponding to all the points that was used to update the graph")
      ("maptype-cloud","With the option save-graph-cloud enabled, this option will use the underlying map type (such as NDT-OM) to create the pointcloud, rather than using the original points");


  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, *argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    return false;
  }

  keep_min_nb_points = vm.count("clear-min-nb-points");
  min_nb_points_set_uniform = vm.count("min-nb-points-set-uniform");
  NDTCell::setParameters(0.1, 8*M_PI/18.,1000, min_nb_points_for_gaussian, !keep_min_nb_points, min_nb_points_set_uniform);


  if(ndt_generic::GetSensorPose(dataset,transl,euler,Tsensor_offset_tf,Tsensor_offset)) {
    std::cout<<"sensor pose from dataset utilized [" << dataset << "]" << endl;
  }

  mapParPtr= GraphFactory::CreateMapParam(map_type_name); //map_type_name
  regParPtr= GraphFactory::CreateRegParam(registration_type_name);
  graphParPtr=GraphMapNavigatorParamPtr(new GraphMapNavigatorParam());
  if(mapParPtr==NULL || regParPtr==NULL || graphParPtr==NULL){
    std::cout<<"Null pointers"<<endl;
    exit(0);
  }

  use_pointtype_xyzir=vm.count("use-pointtype-xyzir");
  use_odometry_source = base_link_id!="";
  if(use_odometry_source)
    odom_type=ndt_offline::WHEEL_ODOM;
  else
    odom_type=ndt_offline::NO_ODOM;

  use_keyframe=!vm.count("disable-keyframe-update");
  if(!use_odometry_source)
    use_keyframe=false;

  visualize = vm.count("visualize");
  filter_cloud = vm.count("filter-cloud");
  step_control = (vm.count("step-control"));
  gt_mapping= vm.count("gt-mapping");

  check_consistency=vm.count("check-consistency");
  alive = vm.count("alive");
  save_map = vm.count("save-map");
  registration2d=!vm.count("registration3d");
  use_gt_data=gt_base_link_id!="";
  init_pose_gt_frame=vm.count("init-pose-gt-frame");

  regParPtr->enable_registration = !vm.count("disable-registration");
  regParPtr->registration2d=registration2d;
  regParPtr->max_rotation_norm=maxRotationNorm;
  regParPtr->max_translation_norm=maxTranslationNorm;
  regParPtr->rotation_registration_delta=rotationRegistrationDelta;
  regParPtr->translation_registration_delta=translationRegistrationDelta;
  regParPtr->sensor_range=max_range;
  regParPtr->map_size_z= map_size_z;
  regParPtr->check_consistency=check_consistency;
  regParPtr->sensor_pose=Tsensor_offset;
  regParPtr->use_initial_guess=!vm.count("disable-odometry");
  mapParPtr->sizez_=map_size_z;
  mapParPtr->max_range_=max_range;
  mapParPtr->min_range_=min_range;
  mapParPtr->store_points=vm.count("store-points");

  graphParPtr->compound_radius=compound_radius;
  graphParPtr->interchange_radius=interchange_radius;
  graphParPtr->alpha=distance_alpha;
  graphParPtr->map_switch_method=GraphMapNavigatorParam::String2SwitchMethod(map_switching_method);
  std::cout<<"use keyframe="<<use_keyframe;
  graphParPtr->use_keyframe=use_keyframe;
  graphParPtr->min_keyframe_dist=min_keyframe_dist;
  graphParPtr->min_keyframe_rot_deg=min_keyframe_dist_rot_deg;
  std::cout<<"keyframe dist"<<graphParPtr->min_keyframe_dist<<endl;
  use_submap=!vm.count("disable-submaps");
  graphParPtr->use_submap=use_submap;
  graphParPtr->Tsensor=Tsensor_offset;

  mapParPtr->enable_mapping_=!vm.count("disable-mapping");
  mapParPtr->sizey_=map_size_xy;
  mapParPtr->sizex_=map_size_xy;

  GetMotionModel(dataset,mot_model);

  cout<<"reg parameters"<<endl;
  if(  NDTD2DRegParamPtr ndt_reg_ptr=boost::dynamic_pointer_cast<NDTD2DRegParam>(regParPtr)){
    ndt_reg_ptr->resolution=resolution;
    ndt_reg_ptr->resolution_local_factor=resolution_local_factor;
    ndt_reg_ptr->matcher2D_ITR_MAX = max_nb_iters;
    ndt_reg_ptr->multires=vm.count("multi-res");
    ndt_reg_ptr->SoftConstraints=vm.count("soft-constraints");
  }
  {
    if(  NDTMapParamPtr ndt_map_ptr=boost::dynamic_pointer_cast<NDTMapParam>(mapParPtr)){
      ndt_map_ptr->resolution_=resolution;
    }
    if (NDTDLMapParamPtr ndt_map_ptr=boost::dynamic_pointer_cast<NDTDLMapParam>(mapParPtr)){
      ndt_map_ptr->resolution = resolution;
    }
  }

  cout<<"finished reg parameters"<<endl;

  //Check if all iputs are assigned
  if (!vm.count("base-name") || !vm.count("dir-name")){
    std::cout << "Missing base or dir names.\n";
    std::cout << desc << "\n";
    return false;
  }
  if (vm.count("help")){
    std::cout << desc << "\n";
    return false;
  }

  use_pointtype_xyzir = vm.count("use-pointtype-xyzir");
  generate_eval_files = vm.count("generate-eval-files");
  use_only_static_scans = vm.count("use-only-static-scans");

  save_used_merged_clouds = vm.count("save-used-merged-clouds");
  maptype_cloud = vm.count("maptype-cloud");
  save_graph_cloud = vm.count("save-graph-cloud");
  cout<<"base-name:"<<base_name<<endl;
  cout<<"dir-name:"<<map_dir_name<<endl;

  return true;


}
void initializeRosPublishers(){
  ros::Time::init();
  srand(time(NULL));
  gt_pub=new ros::Publisher();
  fuser_pub=new ros::Publisher();
  odom_pub=new ros::Publisher();
  cloud_pub=new ros::Publisher();
  *gt_pub    =n_->advertise<nav_msgs::Odometry>("/GT", 50);
  *fuser_pub =n_->advertise<nav_msgs::Odometry>("/fuser", 50);
  *odom_pub = n_->advertise<nav_msgs::Odometry>("/odom", 50);
  *cloud_pub = n_->advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
}
void printParameters(){
  std::cout<<"Output directory: "<<output_dir_name<<endl;
  if(filter_cloud){
    std::cout << "Filtering FOV of sensor to min/max "<<hori_min<<" "<<hori_max<<endl;
    std::cout << "Filtering height of sensor to min/max "<<z_min<<" "<<z_max<<endl;
  }
  else
    std::cout<<"No FOV filter."<<endl;

}
void SaveMap(){
  if( fuser_!=NULL &&fuser_->FramesProcessed()>0){
    char path[1000];
    snprintf(path,999,"%s/%s.map",output_dir_name.c_str(),base_name.c_str());
    if(save_map){
      fuser_->SaveGraphMap(path);
      //fuser_->SaveCurrentNodeAsJFF(path);
    }
    if(save_graph_cloud)
      fuser_->SavePointCloud(path,!maptype_cloud);
  }
}
template<class PointT>
void PlotAll(bool registration_update, pcl::PointCloud<PointT> points2){

  static tf::TransformBroadcaster br;
  gt_pose_msg.header.frame_id="/world";
  fuser_pose_msg.header.frame_id="/world";
  odom_pose_msg.header.frame_id="/world";

  ros::Time t=ros::Time::now();
  //Plot Pose estimate
  tf::Transform tf_fuser_pose;
  tf::poseEigenToTF(fuser_pose,tf_fuser_pose);
  br.sendTransform(tf::StampedTransform(tf_fuser_pose,t,"/world","/fuser_base_link"));

  if(use_odometry_source){
    odom_pose = odom_pose*Todom_base_prev.inverse()*Todom_base;
    tf::Transform tf_odom_pose;
    tf::poseEigenToTF(odom_pose,tf_odom_pose);
    br.sendTransform(tf::StampedTransform(tf_odom_pose,t,"/world","/odom_base_link"));
    odom_pose_msg.header.stamp=t;
    tf::poseEigenToMsg(odom_pose, odom_pose_msg.pose.pose);
    odom_pub->publish(odom_pose_msg);
  }
  if(gt_base_link_id!=""){
    tf::Transform tf_gt_base;
    br.sendTransform(tf::StampedTransform(tf_gt_base,t,   "/world", "/state_base_link"));
  }

  br.sendTransform(tf::StampedTransform(Tsensor_offset_tf,t,"/fuser_base_link","/fuser_laser_link"));
  fuser_pose_msg.header.stamp=t;
  tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);
  Eigen::MatrixXd cov=predCov;
  cov.block(0,0,3,3)=fuser_pose.rotation()*cov.block(0,0,3,3)*fuser_pose.rotation().inverse();
  cov.resize(1,36);

  for(int i=0;i<36;i++)
    fuser_pose_msg.pose.covariance[i]=cov.data()[i];

  gt_pose_msg.header.stamp=t;
  tf::poseEigenToMsg(Tgt_base, gt_pose_msg.pose.pose);
  gt_pub->publish(gt_pose_msg);

  if ( (registration_update && counter%skip_frame==0) ){ //
    fuser_pub->publish(fuser_pose_msg);
    points2.header.frame_id="/world";
    pcl_conversions::toPCL(t, points2.header.stamp);
    Eigen::Affine3d tmp=fuser_pose*Tsensor_offset;
    perception_oru::transformPointCloudInPlace(tmp, points2);
    cloud_pub->publish(points2);
    fuser_->PlotMapType();
  }

}

template<typename PointT>
void processData() {

  n_=new ros::NodeHandle("~"); //ej pekartyp
  fuser_=NULL;

  stringstream ss;
  string name="";
  ss<<name<<dataset<<"_gt="<<gt_mapping<<std::string("_submap=")<<use_submap<<"_sizexy="<<map_size_xy<<"_Z="<<map_size_z<<std::string("_intrchR=")<<interchange_radius<<std::string("_compR=")<<compound_radius<<std::string("_res=") <<resolution<<std::string("_maxSensd=") << max_range<<"_keyF="<<use_keyframe<<"_d="<<min_keyframe_dist<<"_deg="<<min_keyframe_dist_rot_deg<<"_alpha="<<distance_alpha;
  base_name+=ss.str();
  bool dl = (map_type_name == "ndt_dl_map");
  base_name+="_dl="+toString(dl)+"_xyzir="+toString(use_pointtype_xyzir)+"_mpsu="+toString(min_nb_points_set_uniform)+"_mnpfg="+toString(min_nb_points_for_gaussian)+"kmnp"+toString(keep_min_nb_points);
  ndt_generic::CreateEvalFiles eval_files(output_dir_name,base_name,generate_eval_files);

  printParameters();
  initializeRosPublishers();

  std::string tf_interp_link = gt_base_link_id;//base_link_id;
  if(!gt_mapping)
    tf_interp_link = base_link_id;

  std::vector<std::string> ros_bag_paths;
  if(!LocateRosBagFilePaths(map_dir_name,ros_bag_paths)){
    std::cout<<"couldnt locate ros bags"<<endl;
    exit(0);
  }

  counter = 0;

  std::cout<<"opening bag files"<<endl;
  for(int i=0; i<ros_bag_paths.size(); i++) {
    bool initiated=false;
    std::string bagfilename = ros_bag_paths[i];
    std::cout<<"Opening bag file:"<< bagfilename.c_str()<<endl;

    ndt_offline::readPointCloud reader(bagfilename,Tsensor_offset,odom_type,lidar_topic,min_range,max_range,velodyne_config_file,sensor_time_offset,tf_topic,tf_world_frame,tf_interp_link);
    pcl::PointCloud<PointT> cloud, points2;

    ndt_generic::StepControl step_controller;
    bool found_scan=true;

    while(n_->ok()){

      ros::Time t1=ros::Time::now();
      found_scan = reader.readNextMeasurement<PointT>(cloud);

      ros::Time t2=ros::Time::now();
      if(!found_scan)
        continue;

      if(filter_cloud)
        ndt_generic::filter_height_angle(cloud,hori_min,hori_max,z_min,z_max);

      if (cloud.size() == 0) continue; // Check that we have something to work with depending on the FOV filter here...
      points2 = cloud;

      Tgt_base=Todom_base=Eigen::Affine3d::Identity();
      bool odometry_valid=true,gt_valid=true;
      if(use_odometry_source)
        odometry_valid=reader.GetOdomPose(reader.GetTimeOfLastCloud(),base_link_id,Todom_base);
      if(use_gt_data)
        gt_valid=reader.GetOdomPose(reader.GetTimeOfLastCloud(),gt_base_link_id,Tgt_base);

      if(fuser_==NULL){
        if( (use_gt_data &&!gt_valid) || (use_odometry_source && !odometry_valid) ){
          cout<<"Invalid odometry, skipping frame"<<endl;
          counter ++;
          continue;
        }
        if(use_gt_data ){
          Tgt_base_prev = Tgt_base;
          if(use_odometry_source)
            odom_pose=Tgt_base;
        }
        if(use_odometry_source)
          Todom_base_prev = Todom_base;

        fuser_pose=Tgt_base*ndt_generic::xyzrpyToAffine3d(init(0),init(1),init(2),init(3),init(4),init(5) );//specify initial pose offset with respect to GT, (if avaliable), otherwise Identity.

        if(init_pose_gt_frame)
          fuser_=new GraphMapFuser(regParPtr,mapParPtr,graphParPtr,Eigen::Affine3d::Identity(),Tsensor_offset);
        else
          fuser_=new GraphMapFuser(regParPtr,mapParPtr,graphParPtr,fuser_pose*Tsensor_offset,Tsensor_offset);

        fuser_->SetFuserOptions(save_used_merged_clouds);
        fuser_->SetkeyframeOptions(use_only_static_scans);
        fuser_->Visualize(visualize,plotmarker::point/*plotmarker::sphere*/);
        fuser_->SetMotionModel(mot_model);

        std::cout<<fuser_->ToString()<<endl;
        initiated=true;
      }
      else if(!initiated && fuser_!=NULL){
        fuser_->SetFuserPose(fuser_pose);
        initiated=true;
      }

      predCov=Eigen::MatrixXd::Identity(6,6);
      bool registration_update=true;
      Eigen::Affine3d Tmotion=Eigen::Affine3d::Identity();

      if(gt_mapping){
        Tmotion = Tgt_base_prev.inverse()*Tgt_base;
        fuser_pose=Tgt_base_prev;
        cout<<"Tmot_z="<<Tmotion.translation()(2)<<",fuser_z= "<<fuser_pose.translation()(2)<<endl;

        for(int i=0;i<6;i++)
          predCov(i,i)=sigma(i);
        predCov=predCov/lambda;
        cout<<"Predcov=\n"<<predCov<<endl;
      }
      else if(use_odometry_source){
        Tmotion = Todom_base_prev.inverse()*Todom_base;
        /*if(fuser_->FramesProcessed()<=5)
          predCov=fuser_->PredictOdomUncertainty(fuser_pose*Tmotion,registration2d)/(pow(2,fuser_->FramesProcessed())*lambda);
        else*/
          predCov=fuser_->PredictOdomUncertainty(fuser_pose*Tmotion,registration2d)/lambda;
      }

      if (use_only_static_scans)
        registration_update=fuser_->ProcessFrameStaticGlobalMap<PointT>(cloud,fuser_pose,Tmotion);
      else
        registration_update= fuser_->ProcessFrame<PointT>(cloud,fuser_pose,Tmotion,predCov);

      ros::Time t3=ros::Time::now();
      counter++;

      PlotAll<PointT>(registration_update, points2);
      if (registration_update){
        eval_files.Write( reader.GetTimeOfLastCloud(),Tgt_base,Todom_base,fuser_pose, fuser_pose*Tsensor_offset);
      }

      ros::Time t4=ros::Time::now();
      Tgt_base_prev = Tgt_base;
      Todom_base_prev = Todom_base;
      cout<<"read bag= "<<t2-t1<<", process frame= "<<t3-t2<<", plot="<<t4-t3<<endl;
      if(step_control)
        step_controller.Step(counter);
    }
  }

  eval_files.Close();
  SaveMap();

}



/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
/// *!!MAIN!!*
/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
///

int main(int argc, char **argv){
  ros::init(argc, argv, "graph_fuser3d_offline");
  po::options_description desc("Allowed options");

  std::cout<<"Read params"<<endl;
  bool succesfull=ReadAllParameters(desc,argc,&argv);
  if(!succesfull)
    exit(0);

  if (use_pointtype_xyzir) {
    processData<velodyne_pointcloud::PointXYZIR>();
  }
  else {
    processData<pcl::PointXYZ>();
  }




  if (alive) {
    while (1) {
      usleep(1000);
    }
  }
  usleep(1000*1000);
  std::cout << "Done." << std::endl;
}
