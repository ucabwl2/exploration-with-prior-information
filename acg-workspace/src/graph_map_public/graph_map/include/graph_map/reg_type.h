#ifndef REGISTRATIONTYPE_H
#define REGISTRATIONTYPE_H
#include "graphfactory.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "iostream"
#include "string"
#include "stdio.h"
#include <velodyne_pointcloud/point_types.h>
#include "ndt_generic/eigen_utils.h"
#include "graph_map/map_type.h"
#include "ndt_registration/registration.h"
namespace perception_oru{
namespace graph_map{

class registrationParameters{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~registrationParameters()=0;
  virtual void GetParametersFromRos();
  bool enable_registration=true;
  bool registration2d=false;
  bool do_soft_constraints=false;
  bool check_consistency=true;
  bool use_initial_guess=true;
  double max_translation_norm=0.3,max_rotation_norm=M_PI/10.0;
  double translation_registration_delta=0, rotation_registration_delta=0;
  double map_size_z=12;
  double sensor_range=130;
  Eigen::Affine3d sensor_pose;
 // lslgeneric::MotionModel2d motion_model_2d_;
//  lslgeneric::MotionModel3d motion_model_3d_;
protected:
  registrationParameters();
private:
  friend class GraphFactory;
};

class registrationType{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~registrationType()=0;

  /*!
   * \brief RegisterScan Perform scan-to-map registration
   * \param maptype The map in the selected scan-to-map registration method
   * \param Tnow Prediction and registration result
   * \param cloud pointcloud
   * \param Tcov Covariance input and/or output
   * \return false only if registration was performed and failed
   */
  bool RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::MatrixXd &Tcov);

  bool RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud);

  bool RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, Eigen::MatrixXd &Tcov);

  bool RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud);

  virtual bool RegisterMap2Map(MapTypePtr map_prev,MapTypePtr map_next, Eigen::Affine3d &Tdiff,double &match_score){}

  virtual std::string ToString();

  void visualize(bool viz,const Eigen::Affine3d &offset){offset_=offset; visualize_=viz;}

//  static void SetOutput(bool disabled){disable_output=disabled;}
  bool enable_registraiton=true;
  bool registration2d_=true;
  bool do_soft_constraints_=false;
  bool check_consistency=true;
  bool use_initial_guess=true;
  double max_translation_norm=0.6,max_rotation_norm=3*M_PI/180;
  double translation_registration_delta=0, rotation_registration_delta=0;
  double sensor_range=100;
  double map_size_z;
  unsigned int  failed_registrations=0;
  unsigned int  succesfull_registrations=0;
  Eigen::Affine3d sensorPose_=Eigen::Affine3d::Identity();//Translation between robot and sensor frame
//  static bool disable_output;
protected:

  registrationType(RegParamPtr regparam);

  virtual bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov){std::cerr<<"No registration for pcl::PointXYZ implemented"<<std::endl;}//This methods attempts to register the point cloud versus the map using the affine transformation guess "Tm"

  virtual bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,Eigen::MatrixXd &Tcov){std::cerr<<"No registration for velodyne_pointcloud::PointXYZIR implemented"<<std::endl;}

  bool visualize_=false;
  Eigen::Affine3d offset_;
  regStatus status_=SUCCESS; //Contain the status of the registration outcome

private:
  friend class GraphFactory;

};



}

}
#endif // REGISTRATIONTYPE_H
