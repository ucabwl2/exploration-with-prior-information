#ifndef D2D_NDT_H
#define D2D_NDT_H
#include "graph_localization/localization_factory.h"//must be included first
#include "graph_localization/localization_type.h"
#include "graph_map/ndt/ndtd2d_reg_type.h"
#include "graph_map/ndt/ndt_map_type.h"
#include "graph_map/graphfactory.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ndt_localization/3d_ndt_mcl.hpp"
#include <angles/angles.h>
#include <velodyne_pointcloud/point_types.h>
#include "stdio.h"
#include "vector"

namespace perception_oru{
namespace graph_localization{

class RegLocalisationType:public LocalizationType{

public:

  RegLocalisationType(LocalisationParamPtr param);

  ~RegLocalisationType(){}

  std::string ToString();

  void InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance); //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

  bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion);

  bool UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion);

protected:

  double getDoubleTime();

  void NormaliseEulerAngles(Eigen::Vector3d &euler);

  int counter=0;
  bool keyframe_update_=false;
  double min_keyframe_dist_=0.1;
  double min_keyframe_dist_rot_deg_=1;
  RegTypePtr regptr_;

  Eigen::Affine3d pose_last_update_;
  std::vector<double> motion_model, motion_model_offset;

private:
  friend class LocalisationFactory;
};
class RegLocalisationParam:public LocalisationParam{
public:
  double mapSizeZ=0;
  double sensorRange=0;
  bool keyframe_update=false;
  double min_keyframe_dist=0.1;
  double min_keyframe_dist_rot_deg=1;
  RegParamPtr registration_parameters;
  std::vector<double> motion_model, motion_model_offset;
  void GetParamFromRos(){}
  RegLocalisationParam();
  ~RegLocalisationParam(){}
private:
  friend class LocalisationFactory;
};
}
}
#endif // D2D_NDT_H
