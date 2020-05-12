#ifndef GRAPH_MAP_FUSER_H
#define GRAPH_MAP_FUSER_H
#include "stdio.h"
#include "iostream"
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/reg_type.h"
#include "ndt/ndtd2d_reg_type.h"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
//#include "gnuplot-iostream.h"
#include "ndt_map/ndt_map.h"
#include "ndt_generic/motion_model_2d.h"
#include "ndt_generic/motion_model_3d.h"
#include "graph_map/graph_plot.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "boost/serialization/shared_ptr.hpp"
//#include "ndt_generic/eigen_utils.h"
#include "ros/time.h"
namespace perception_oru{
namespace graph_map{
using namespace std;
using namespace perception_oru;
using perception_oru::MotionModel2d;
using perception_oru::MotionModel3d;
using Eigen::Affine3d;
class GraphMapFuser{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW



    GraphMapFuser(){}

  GraphMapFuser(string maptype, string registratorType, const Eigen::Affine3d &init_pose, const Affine3d &sensorPose);//Ros friendly constructor to read parameters from ros parameter server

  GraphMapFuser(RegParamPtr regParam,  MapParamPtr mapParam, GraphMapNavigatorParamPtr graph_param, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose);

  Affine3d GetPoseLastFuse() const{return pose_last_fuse_;}

  void SaveGraphMap(const std::string &filename);

  void SavePointCloud(const string &filename, bool original_points=true);

  void SaveCurrentNodeAsJFF (const  std::string &filename);

  void SetMotionModel(const  MotionModel3d &motion_model){motion_model_3d_=motion_model;}

  void SetFuserPose(const Eigen::Affine3d & pose){pose_last_fuse_=pose;}

    const GraphMapNavigatorPtr& GetGraph() const {return graph_map_;}
    GraphMapNavigatorPtr& GetGraph() {return graph_map_;}

  Eigen::MatrixXd PredictOdomUncertainty(const Eigen::Affine3d Tnow, bool prediction2d=true);

  //!
  //! //! \brief ProcessFrame
  //! //! \param cloud
  //! //! \param Tnow
  //! //! \param Tmotion robot movement
  //! //! \return true if registration was succesfull and map was updated
  //! bool ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow, Eigen::Affine3d &Tmotion); //cloud is the current scan in robot frame,  Tnow is the current pose in world frame

  template<class PointT> bool  ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion);

  template<class PointT> bool  ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion, Eigen::MatrixXd &Tcov);

  template<class PointT>  bool ProcessFrameStatic(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion);

  template<class PointT>  bool ProcessFrameStaticGlobalMap(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion);

  template<class PointT>  void UpdateMultipleMaps(pcl::PointCloud<PointT> &cloud,Eigen::Affine3d &Tnow);

  template<class PointT>  void UpdateSingleMap(pcl::PointCloud<PointT> &cloud,Eigen::Affine3d &Tnow);

  template<class PointT>  bool scanmatching(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow, Eigen::Affine3d &Tmotion);

  unsigned int FramesProcessed() const{return nr_frames_;}

  void PlotMapType();

  bool ErrorStatus(string status="");

  bool FuseFrame(const Affine3d &Tnow, const Affine3d &Tmotion);

  bool KeyFrameBasedFuse(const Affine3d &Tnow );

  bool FuseNoMotionFrames(const Affine3d &Tnow ,const Affine3d &Tmotion);

  void Visualize(bool enableVisualOutput,plotmarker marker=plotmarker::sphere);

  void SetFuserOptions( bool save_merged_cloud = false);

  std::string ToString();

  void SetkeyframeOptions(bool static_scans_only){fuse_no_motion_frames=static_scans_only;}

protected:

  void plotGTCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud);

  ros::NodeHandle n_;
  string maptype_,registratorType_;
  Eigen::Affine3d initPose_,sensorPose_,pose_last_fuse_;
  GraphMapNavigatorPtr graph_map_;
  GraphMapNavigatorParamPtr graph_param_;
  MapParamPtr mapParam_;
  RegParamPtr regParam_;
  RegTypePtr registrator_;
  MotionModel3d motion_model_3d_;
  plotmarker marker_;
  bool initialized_=false;

  // Parameter - not used
  bool visualize_=false;

  bool use_keyframe_=true;

  // Parameter
  bool gt_mapping_enabled_=false;

  // Parameter retreived from graphParam
  double min_keyframe_dist_=0.5;
  double min_keyframe_rot_deg_=15;

  // Counter - not used
  unsigned int nr_frames_=0;

  //Needed because otherwise, the first node added tries to do registration which fails, and the mapping only starts at the second node
  bool first_map_initialised = false;

  bool fuse_no_motion_frames=false;
  bool multiple_map_update=false;
  bool avoid_lidar_shadow=false;
  bool use_scanmatching=false;
  bool scanmatching_extrapolation=false;
  bool save_merged_cloud_=false;
  bool observationInRobotFrame=false;


};

}
}
#include <graph_map/graph_map_fuser_impl.h>

#endif // GRAPH_MAP_FUSER_H
