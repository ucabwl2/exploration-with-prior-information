#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>
#include <ros/ros.h>
#include "ndt_offline/readpointcloud.h"
#include "eigen3/Eigen/Eigen"
#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace std;
using namespace ndt_offline;
int main(int argc, char **argv){
  ros::init(argc, argv, "graph_fuser3d_offline");
  string velodyne_config_file;
  std::string bag_file_path;
  std::string lidar_topic;
  std::string interp_link;
  std::string odom_type;
  ndt_offline::OdometryType odom;
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("bag-file" ,po::value<string>(&bag_file_path)->default_value(std::string("")) ,"bag file")
      ("velodyne-config-file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("../config/velo32.yaml")), "configuration file for the scanner")
      ("lidar-topic", po::value<std::string>(&lidar_topic)->default_value(std::string("/velodyne_packets")), "topic for velodyne topic")
      ("interpolation-link-id", po::value<std::string>(&interp_link)->default_value(std::string("odom_base_link")), "topic for velodyne topic")
      ("odom-type", po::value<std::string>(&odom_type)->default_value(std::string("")), "odom_type")
      ;


  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  if (vm.count("help")){
    cout << desc << "\n";
    return 1;
  }
  Eigen::Affine3d Tsens=Eigen::Affine3d::Identity();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cout<<"Opening reader at path: "<<bag_file_path<<endl;
  if(odom_type=="WHEEL_ODOM")
    odom=WHEEL_ODOM;
  else if(odom_type=="IMU")
    odom=IMU;
  else
    odom=NO_ODOM;

  ndt_offline::readPointCloud *reader=new ndt_offline::readPointCloud(bag_file_path, Tsens,odom,lidar_topic,0.6,130,velodyne_config_file,0.0,"/tf","/world",interp_link);
  int counter=0;
  while(reader->readNextMeasurement(cloud) && ros::ok()){
    cout<<"Counter="<<counter++<<", Cloud size="<<cloud.size()<<endl;
    //usleep(1000*300);
  }
  cout<<"end of bag"<<endl;





}
