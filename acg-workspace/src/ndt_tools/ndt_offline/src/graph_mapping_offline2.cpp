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
#include "ndt_offline/imu_prediction.h"
#include <unistd.h>
#include "tf_conversions/tf_eigen.h"
#include "ndt_generic/io.h"
#include "ndt_offline/readpointcloud.h"
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
bool use_imu_prediction=false;
double lambda=1.0;
bool visualize=true;
bool use_multires=false;
bool beHMT=false;
bool filter_fov=false;
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
perception_oru::MotionModel2d::Params motion_params;
std::string base_link_id="", gt_base_link_id="", tf_world_frame="";
std::string velodyne_config_file="";
std::string lidar_topic="";
std::string velodyne_frame_id="";
std::string map_type_name="",registration_type_name="";
std::string tf_topic="";
tf::Transform tf_sensor_pose;
std::string map_switching_method="";
Eigen::Affine3d sensor_offset,fuser_pose, odom_pose;//Mapping from base frame to sensor frame
ros::NodeHandle *n_=NULL;
RegParamPtr regParPtr=NULL;
MapParamPtr mapParPtr=NULL;
GraphMapNavigatorParamPtr graphParPtr=NULL;
ndt_offline::imu_prediction *imu_prediction=NULL;
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
unsigned int skip_frame=20;
ros::Publisher *gt_pub,*fuser_pub,*cloud_pub,*odom_pub;
nav_msgs::Odometry gt_pose_msg,fuser_pose_msg,odom_pose_msg;
//VelodyneBagReader<pcl::PointXYZ> *vreader;
//PointCloudBagReader<pcl::PointXYZ> *preader;
//ReadBagFileGeneric<pcl::PointXYZ> *reader;
bool use_pointtype_xyzir;
int min_nb_points_for_gaussian;
bool keep_min_nb_points;
bool min_nb_points_set_uniform;
int nb_measurements=1;
int max_nb_iters=30;
bool generate_eval_files = false;
bool save_used_clouds = false;
bool use_only_static_scans = false;
bool save_used_merged_clouds = false;
bool save_graph_cloud=false;
bool maptype_cloud=false;
int nb_frames = 0;
bool robotFrameSubmap=false;
bool disable_undistortion=false;
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
bool GetSensorPose(const std::string &dataset,  Eigen::Vector3d & transl,  Eigen::Vector3d &euler,tf::Transform &tf_sensor){

  tf::Quaternion quat;

  bool found_sensor_pose=false;
  if(dataset.compare("oru-field")==0){
    transl[0]=0;
    transl[1]=0;
    transl[2]=0;
    euler[0]=-0.00341974;
    euler[1]=0.00417214;
    euler[2]=-1.56988;
    found_sensor_pose=true;
    //    -0.0344436 --Euler-x -0.00341974 --Euler-y 0.00417214 --Euler-z -1.56988
  }
  else if(dataset.compare("oru-basement")==0){
    transl[0]=0.3;
    transl[1]=0;
    transl[2]=1.3;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.57;
    found_sensor_pose=true;
  }
  else if(dataset.compare("default")==0){
    transl[0]=0;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=0;
    found_sensor_pose=true;
  }
  else if(dataset.compare("arla-2012")==0){
    transl[0]=1.17920618535632;
    transl[1]= -0.285884882359476;
    transl[2]=2.0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.58804135060281;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("coop-2013")==0){
    transl[0]=0.959 ;
    transl[1]= 0.343;
    transl[2]=0.022 ;
    euler[0]=0.0;//0.0038;
    euler[1]=0.0;
    euler[2]=0.121;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("kitti")==0){ //car center to lidar
    transl[0]=-0.27;
    transl[1]= 0;
    transl[2]=0.8;
    euler[0]=0;
    euler[1]=0;
    euler[2]=0;//-1.625
    found_sensor_pose=true;
  }
  else if(dataset.compare("volvo_2017_12_01")==0){
    transl[0]= 0.270054;
    transl[1]= -0.000547494;
    transl[2]=3.79621;
    euler[0]=-0.00697922;
    euler[1]=-0.00933762;
    euler[2]=-3.11258 ;
    found_sensor_pose=true;
  }
  quat.setRPY(euler[0], euler[1], euler[2]);
  tf::Vector3 trans(transl[0], transl[1], transl[2]);
  tf_sensor  = tf::Transform(quat,trans);
  tf::poseTFToEigen(tf_sensor,sensor_offset);
  cout<<"sensor pose:\n"<<sensor_offset.translation().transpose()<<endl;
  return found_sensor_pose;
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
      ("map-type-name", po::value<string>(&map_type_name)->default_value(std::string("default")), "type of map to use e.g. ndt_map or ndt_dl_map (default it default)")
      ("registration-type-name", po::value<string>(&registration_type_name)->default_value(std::string("ndt_d2d_reg")), "type of map to use e.g. ndt_d2d_reg or ndt_dl_reg (default it default)")
      ("visualize", "visualize the output")
      ("no-odometry", "dont use odometry")
      ("imu-prediction", "dont use odometry")
      ("disable-mapping", "build maps from cloud data")
      ("attempts", po::value<int>(&attempts)->default_value(1), "Total retries of localisation, can be used to generate multiple files")
      ("step-control", "use step control in the optimization (default=false)")
      ("base-name", po::value<string>(&base_name), "prefix for all generated files")
      ("reader-type", po::value<string>(&bag_reader_type)->default_value("velodyne_reader"), "e.g. velodyne_reader or pcl_reader")
      ("data-set", po::value<string>(&dataset)->default_value(std::string("")), "choose which dataset that is currently used, this option will assist with assigning the sensor pose")
      ("dir-name", po::value<string>(&map_dir_name), "where to look for ros bags")
      ("map-switching-method", po::value<string>(&map_switching_method)->default_value("node_position"), "where to look for ros bags")
      ("output-dir-name", po::value<string>(&output_dir_name)->default_value("/home/daniel/.ros/maps"), "where to save the pieces of the map (default it ./map)")
      ("map-size-xy", po::value<double>(&map_size_xy)->default_value(83.), "size of submaps")
      ("map-size-z", po::value<double>(&map_size_z)->default_value(6.0), "size of submaps")
      ("itrs", po::value<int>(&itrs)->default_value(30), "number of iteration in the registration")
      ("fuse-incomplete", "fuse in registration estimate even if iterations ran out. may be useful in combination with low itr numbers")
      ("filter-fov", "cutoff part of the field of view")
      ("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
      ("hori-min", po::value<double>(&hori_min)->default_value(-hori_max), "the minimum field of view angle horizontal")
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
      ("velodyne-packets-topic", po::value<std::string>(&lidar_topic)->default_value(std::string("/velodyne_packets")), "velodyne packets topic used")
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
      ("nickes-arg", po::value<double>(&nickes_arg)->default_value(0.), "nicke har ett viktigt argument")
      ("lambda-sc", po::value<double>(&lambda)->default_value(1.), "nicke har ett viktigt argument")
      ("skip-frame", po::value<unsigned int>(&skip_frame)->default_value(20), "sframes to skip before plot map etc.")
      ("sensor-time-offset", po::value<double>(&sensor_time_offset)->default_value(0.), "timeoffset of the scanner data")
      ("registration3d","registration3d")
      ("disable-undistortion","dont undistort points using IMU")
      ("disable-registration", "Disable Registration")
      ("soft-constraints", "Use soft constraints in the registration")
      ("check-consistency", "if consistency should be checked after registration")("multi-res", "multi resolution registration")
      ("consistency-max-rot",po::value<double>(&maxRotationNorm)->default_value(0.78539816339),"maxRotationNorm")
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
      ("save-used-clouds", "save the point cloud at each used scan pose")
      ("save-used-merged_clouds", "incase of using static scans, multiple scans is used for each update, this will save the merged clouds")
      ("store-points","store pointclouds used to create the graph_map")
      ("save-graph-cloud", "Save a pointcloud corresponding to all the points that was used to update the graph")
      ("maptype-cloud","With the option save-graph-cloud enabled, this option will use the underlying map type (such as NDT-OM) to create the pointcloud, rather than using the original points")
      ("robot-frame-submap", "all submapping (deploy new submap) is done in robot frame");

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


  if(GetSensorPose(dataset,transl,euler,tf_sensor_pose)) {
    std::cout<<"sensor pose from dataset utilized [" << dataset << "]" << endl;
  }

  mapParPtr= GraphFactory::CreateMapParam(map_type_name); //map_type_name
  regParPtr= GraphFactory::CreateRegParam(registration_type_name);

  graphParPtr=GraphMapNavigatorParamPtr(new GraphMapNavigatorParam());
  if(mapParPtr==NULL || regParPtr==NULL || graphParPtr==NULL){
    std::cout<<"null pointers"<<endl;
    return false;

  }
  use_imu_prediction= vm.count("imu-prediction");
  use_odometry_source = !vm.count("no-odometry") &&!use_imu_prediction;
  visualize = vm.count("visualize");
  filter_fov = vm.count("filter-fov");
  step_control = (vm.count("step-control"));
  gt_mapping= vm.count("gt-mapping");
  use_keyframe=!vm.count("disable-keyframe-update");
  robotFrameSubmap=vm.count("robot_frame_submap");
  check_consistency=vm.count("check-consistency");
  alive = vm.count("alive");
  save_map = vm.count("save-map");
  registration2d=!vm.count("registration3d");


  regParPtr->enable_registration = !vm.count("disable-registration") && !gt_mapping;
  regParPtr->registration2d=registration2d;
  regParPtr->max_rotation_norm=maxRotationNorm;
  regParPtr->max_translation_norm=maxTranslationNorm;
  regParPtr->rotation_registration_delta=rotationRegistrationDelta;
  regParPtr->translation_registration_delta=translationRegistrationDelta;
  regParPtr->sensor_range=max_range;
  regParPtr->map_size_z= map_size_z;
  regParPtr->check_consistency=check_consistency;
  regParPtr->sensor_pose=sensor_offset;
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

  mapParPtr->enable_mapping_=!vm.count("disable-mapping");
  mapParPtr->sizey_=map_size_xy;
  mapParPtr->sizex_=map_size_xy;


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
  save_used_clouds = vm.count("save-used-clouds");
  use_only_static_scans = vm.count("use-only-static-scans");
  disable_undistortion=vm.count("disable-undistortion");

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
  if(filter_fov)
    std::cout << "Filtering FOV of sensor to min/max "<<hori_min<<" "<<hori_max<<endl;
  else
    std::cout<<"No FOV filter."<<endl;

}
std::string boolToString(bool input){
  return input?std::string("true"):std::string("false");
}

template<typename PointT>
void processData() {

  n_=new ros::NodeHandle("~"); //ej pekartyp

  GraphMapFuser *fuser_=NULL;


  stringstream ss;
  string name="";
  ss<<name<<dataset<<"_gt="<<gt_mapping<<std::string("_submap=")<<use_submap<<"_sizexy="<<map_size_xy<<"_Z="<<map_size_z<<std::string("_intrchR=")<<interchange_radius<<std::string("_compR=")<<compound_radius<<std::string("_res=") <<resolution<<std::string("_maxSensd=") << max_range<<"_keyF="<<use_keyframe<<"_d="<<min_keyframe_dist<<"_deg="<<min_keyframe_dist_rot_deg<<"_alpha="<<distance_alpha;
  base_name+=ss.str();
  bool dl = (map_type_name == "ndt_dl_map");
  base_name+="_dl="+toString(dl)+"_xyzir="+toString(use_pointtype_xyzir)+"_mpsu="+toString(min_nb_points_set_uniform)+"_mnpfg="+toString(min_nb_points_for_gaussian)+"kmnp"+toString(keep_min_nb_points);
  ndt_generic::CreateEvalFiles eval_files(output_dir_name,base_name,generate_eval_files);

  printParameters();
  initializeRosPublishers();
  tf::TransformBroadcaster br;
  gt_pose_msg.header.frame_id="/world";
  fuser_pose_msg.header.frame_id="/world";
  odom_pose_msg.header.frame_id="/world";

  std::string tf_interp_link = base_link_id;
  if(gt_mapping)
    tf_interp_link = gt_base_link_id;


  /// Set up the sensor link
  tf::StampedTransform sensor_link; ///Link from /odom_base_link -> velodyne
  sensor_link.child_frame_id_ = velodyne_frame_id;
  sensor_link.frame_id_ = tf_interp_link;//tf_base_link; //"/odom_base_link";
  sensor_link.setData(tf_sensor_pose);

  std::vector<std::string> ros_bag_paths;
  if(!LocateRosBagFilePaths(map_dir_name,ros_bag_paths)){
    std::cout<<"couldnt locate ros bags"<<endl;
    exit(0);
  }

  int counter = 0;

  std::cout<<"opening bag files"<<endl;
  for(int i=0; i<ros_bag_paths.size(); i++) {
    std::string bagfilename = ros_bag_paths[i];
    std::cout<<"Opening bag file:"<< bagfilename.c_str()<<endl;


    ndt_offline::readPointCloud pointcloud_reader(bagfilename, sensor_offset,"velodyne","imu","/velodyne_packets",0.4,130,velodyne_config_file,0.0,!disable_undistortion);//(bag_file_path,T_lidar,"velodyne","imu","/velodyne",0.6,130,velodyne_config_file,Ts);

    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_nofilter, points2;
    tf::Transform tf_scan_source;
    tf::Transform tf_gt_base;
    ros::Time itr=ros::Time::now();
    ros::Time itr_end=itr;
    ndt_generic::StepControl step_controller;

    bool found_scan=true;
    while(found_scan){
      //end_of_bag_file=  vreader.readMultipleMeasurements(nb_scan_msgs,cloud_nofilter,tf_scan_source,tf_gt_base,tf_interp_link);
      //found_scan= preader.readNextMeasurement(cloud_nofilter);
      ros::Time t1=ros::Time::now();
      found_scan= pointcloud_reader.readNextMeasurement(cloud_nofilter);
      ros::Time t2=ros::Time::now();
      if(!n_->ok())
        exit(0);

      cout<<"counter="<<counter<<endl;
      itr=ros::Time::now();

      if(cloud_nofilter.size()==0) continue;

      if(filter_fov) {
        ndt_generic::filter_fov_fun(cloud,cloud_nofilter,hori_min,hori_max);
      } else {
        cloud = cloud_nofilter;
      }

      if (cloud.size() == 0) continue; // Check that we have something to work with depending on the FOV filter here...

      points2 = cloud;

      if(counter == 0){

        if( found_scan!=true){
          std::cout<<"Problem with bag filel"<<endl;
          exit(0);
        }
        //std::cout<<"counter=0"<<endl;
        counter ++;
        cloud.clear();
        cloud_nofilter.clear();
        continue;
      }
      if(counter == 1){
        fuser_pose=Eigen::Affine3d::Identity();

        if(fuser_==NULL){
          cout<<"create fuser"<<endl;
          //
          fuser_=new GraphMapFuser(regParPtr,mapParPtr,graphParPtr,fuser_pose,Eigen::Affine3d::Identity());
          if(use_imu_prediction)
            imu_prediction=new ndt_offline::imu_prediction(bagfilename, sensor_offset,n_,fuser_pose,pointcloud_reader.GetTimeOfLastCloud());


          fuser_->SetFuserOptions(gt_mapping, save_used_merged_clouds);
          fuser_->SetkeyframeOptions(use_only_static_scans);
        }
        else{
          fuser_->SetFuserPose(Eigen::Affine3d::Identity());

        }
        std::cout<<"----------------------PARAMETERS FOR MAPPING--------------------------"<<endl;
        std::cout<<fuser_->ToString()<<endl;
        std::cout<<"----------------------PARAMETERS FOR MAPPING--------------------------"<<endl;
        fuser_->Visualize(visualize,plotmarker::point/*plotmarker::sphere*/);
        counter ++;
        cloud.clear();
        cloud_nofilter.clear();
        continue;
      }
      Eigen::MatrixXd predCov=Eigen::MatrixXd::Identity(6,6);


      if( found_scan!=true)
        break;
      bool registration_update=true;
      Eigen::Affine3d Tmotion=Eigen::Affine3d::Identity();


       if(use_imu_prediction){ //insert IMU prediction here..
          Eigen::Matrix<double, 6, 1> var;
          var<<100,100,100,0.1,0.1,0.15;
          predCov= Eigen::MatrixXd::Identity(6,6)*var.transpose();
          std::cout<<"using imu prediction"<<std::endl;
        //  Tmotion=imu_prediction->PredictPoseAndVelocity(reader.getTimeStampOfLastSensorMsg().toSec());
          //cout<<"Tmotion:\n"<<Tmotion.translation().transpose()<<endl;
        }

        if (use_only_static_scans)
          registration_update=fuser_->ProcessFrameStaticGlobalMap<pcl::PointXYZ>(cloud,fuser_pose,Tmotion);
        else
          registration_update= fuser_->ProcessFrame<pcl::PointXYZ>(cloud,fuser_pose,Tmotion,predCov);


        if(imu_prediction && registration_update)
          imu_prediction->update_registered_pose(fuser_pose,pointcloud_reader.GetTimeOfLastCloud());
        cout<<"updated"<<endl;

      ros::Time t3=ros::Time::now();
      counter++;
      ros::Time tplot=ros::Time::now();
      tf::Transform tf_fuser_pose;
      tf::poseEigenToTF(fuser_pose,tf_fuser_pose);
      br.sendTransform(tf::StampedTransform(tf_fuser_pose,tplot,"/world","/fuser_base_link"));

      fuser_pose_msg.header.stamp=tplot;
      tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);

      predCov.block(0,0,3,3)=fuser_pose.rotation()*predCov.block(0,0,3,3)*fuser_pose.rotation().inverse();
      predCov.resize(1,36);
      for(int i=0;i<36;i++)
        fuser_pose_msg.pose.covariance[i]=predCov.data()[i];

      fuser_pub->publish(fuser_pose_msg);
      gt_pose_msg.header.stamp=tplot;

      if (visualize && (registration_update || counter%skip_frame==0) ){ //
        points2.header.frame_id="/world";
        pcl_conversions::toPCL(tplot, points2.header.stamp);
        perception_oru::transformPointCloudInPlace(fuser_pose, points2);
        cloud_pub->publish(points2);
        fuser_->PlotMapType();
        //GraphPlot::PlotObservationVector(fuser_->GetGraph());
      }
      ros::Time t4=ros::Time::now();

      // GT, odometry and fuser in vehicle frame, estimated sensor pose is the fuser + sensor pose.
      // The motivation is that in case we have estimated the sensor pose accuratly the fuser pose is given in the same origin as GT and Odometry.
      // In case we want to perform some calibration, then the fuser_pose is not related to GT or Odometry, hence we need to find the sensor_offset
      // however, the sensor_offset could be set to some rough initial estimate for the initial estimate to have resonable values and in this case
      // we also store fuser and fuser * sensor_pose to always be able to better tune the sensor_pose estimate.
      cout<<"read bag= "<<t2-t1<<", process frame= "<<t3-t2<<", plot="<<t4-t3<<endl;
      if (registration_update) {
        Eigen::Affine3d tmp_gt=fuser_pose;


        tmp_gt.translation()=tmp_gt.translation()+Eigen::Vector3d( ((double)std::rand())/((double)RAND_MAX)-0.5,((double)std::rand())/((double)RAND_MAX)-0.5,((double)std::rand())/((double)RAND_MAX)-0.5);
        //eval_files.Write( pointcloud_reader.GetTimeOfLastCloud(),tmp_gt,fuser_pose,fuser_pose, fuser_pose);

        if (save_used_clouds) {
          // Save the cloud
          pcl::io::savePCDFileASCII ("cloud"+ndt_generic::toString(nb_frames++)+".pcd", cloud);
        }
      }
      cloud.clear();
      cloud_nofilter.clear();
      itr_end=ros::Time::now();

      if(step_control)
        step_controller.Step(counter);
    }
  }

  eval_files.Close();
  if( fuser_!=NULL &&fuser_->FramesProcessed()>0){
    char path[1000];
    snprintf(path,999,"%s/%s.map",output_dir_name.c_str(),base_name.c_str());
    if(save_map){
      fuser_->SaveGraphMap(path);

    }
    if(save_graph_cloud)
      fuser_->SavePointCloud(path,!maptype_cloud);
  }
}


/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
/// *!!MAIN!!*
/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
///

int main(int argc, char **argv){
  ros::init(argc, argv, "graph_fuser3d_offline2");
  po::options_description desc("Allowed options");

  std::cout<<"Read params"<<endl;
  bool succesfull=ReadAllParameters(desc,argc,&argv);
  if(!succesfull)
    exit(0);

  if (use_pointtype_xyzir)
    processData<velodyne_pointcloud::PointXYZIR>();
  else
    processData<pcl::PointXYZ>();





  if (alive) {
    while (1) {
      usleep(1000);
    }
  }
  usleep(1000*1000);
  std::cout << "Done." << std::endl;
}
