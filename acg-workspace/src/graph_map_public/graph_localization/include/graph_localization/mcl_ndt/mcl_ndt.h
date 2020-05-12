#ifndef MCL_NDT_H
#define MCL_NDT_H
#include "graph_localization/localization_factory.h"//must be included first
#include "graph_localization/localization_type.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ndt_localization/3d_ndt_mcl.hpp"
#include "ndt_localization/3d_particle_filter.hpp"
#include <angles/angles.h>
#include <velodyne_pointcloud/point_types.h>
#include "stdio.h"
#include "vector"
#include "time.h"
#include "ndt_generic/eigen_utils.h"
#include "ndt_generic/motion_model_3d.h"
namespace perception_oru{
namespace graph_localization{

class MCLNDTType:public LocalizationType{

public:

  MCLNDTType(LocalisationParamPtr param);

  ~MCLNDTType(){}

  std::string ToString();

  void InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance); //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}

  bool UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion);

  bool UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion);

  void GetParamFromRos();


protected:

  geometry_msgs::PoseArray ParticlesToMsg(std::vector<particle> &particles);

  void OdometryPrediction(const Eigen::Affine3d &Tmotion, bool disable_noise);

  void ComputeMotionCovar(const Eigen::Affine3d &Tmotion, Eigen::Matrix<double,6,1> &motion_cov );

  void Subsample(std::vector<perception_oru::NDTCell*> &input, std::vector<perception_oru::NDTCell*> &output);

  void AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts);

  void SIResampling();

  NDTMap* GetCurrentNodeNDTMap();

  inline double getDoubleTime();

  inline void normalizeEulerAngles(Eigen::Vector3d &euler);

  particle_filter_3d pf;
  NDTMap * map_;
  Eigen::Affine3d pose_last_update_;
  //std::vector<double> motion_model, motion_model_offset;
  MotionModel3d motion_model_;
  ros::Publisher part_pub;
  int counter=0;//ok
  int sinceSIR_=0; //ok
  int n_particles_=250;
  int SIR_max_iters_wo_resampling_=25;
  bool initialized_=false;
  bool forceSIR=false;//ok
  double resolution=0.5;
  double resolution_sensor=0.5;//ok
  double subsample_level_=1.0;
  double z_filter_min=-10000.0;
  double score_cell_weight=0.1;
  double SIR_varP_threshold=0.6;


private:
  friend class LocalisationFactory;
};
class MCLNDTParam:public LocalisationParam{
public:
  MCLNDTParam();

  ~MCLNDTParam(){}

  void GetParamFromRos();

  std::string ToString();

  bool forceSIR=false;
  double z_filter_min=-10000.0;
  double score_cell_weight=0.1;
  double SIR_varP_threshold=0.6;
  int n_particles=250;
  int SIR_max_iters_wo_resampling=30;
  MotionModel3d motion_model;
  //std::vector<double> motion_model, motion_model_offset;
private:
  friend class LocalisationFactory;
};
}
}
#endif // MCL_NDT_H
