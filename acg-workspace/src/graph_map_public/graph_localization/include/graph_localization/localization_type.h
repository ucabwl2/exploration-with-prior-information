#ifndef LOCALISATIONTYPE_H
#define LOCALISATIONTYPE_H
#include "graph_localization/localization_factory.h"//must be included first
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/graph_map.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <velodyne_pointcloud/point_types.h>
namespace perception_oru{
namespace graph_localization{
using namespace graph_map;



class LocalizationType
{
public:
  LocalizationType(LocalisationParamPtr param);

  virtual ~LocalizationType()=0;

  virtual std::string ToString();

  virtual void InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance)=0; //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

  virtual bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion)=0;

  virtual bool UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion)=0;

  void SetPose( const Eigen::Affine3d &pose);

  Eigen::Affine3d GetPose(){return pose_;}//return pose of robot in world frame

  Eigen::Affine3d GetVelocity();

  void visualize(bool viz){visualize_=viz; }

  bool Initialized(){return initialized_;}

protected:

  bool initialized_=false;
  bool visualize_=false;
  bool localisation2D_=true;
  bool enable_localisation_=true;
  bool no_motion_localisaton_disable=false;
  unsigned int n_obs_search_=5;
  Eigen::Affine3d sensor_pose_;
  GraphMapNavigatorPtr graph_map_;
  LocalisationParamPtr param_;//holds the parameters for localisation, can be used to initialize or Reinitialize

private:
  Eigen::Affine3d pose_=Eigen::Affine3d::Identity();
  Eigen::Affine3d prev_pose_=Eigen::Affine3d::Identity();
  Eigen::Affine3d velocity_=Eigen::Affine3d::Identity();
  friend class LocalisationFactory;
};



class LocalisationParam{
public:

  LocalisationParam();

  ~LocalisationParam();

  virtual void GetParamFromRos();

  virtual std::string ToString();

  GraphMapNavigatorPtr graph_map_;
  bool visualize=false;
  bool enable_measurments=false;
  bool localisation2D=true;
  bool enable_localisation=true;
  unsigned int n_obs_search=5;
  MapSwitchingMethod switch_map_method=node_position;
  Eigen::Affine3d sensor_pose;
private:
  friend class LocalisationFactory;
};


}

}
#endif // LOCALISATIONTYPE_H
