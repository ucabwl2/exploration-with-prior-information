#include "graph_map/graph_map_fuser.h"
namespace perception_oru{
namespace graph_map{
using namespace std;
GraphMapFuser::GraphMapFuser(string maptype, string registratorType, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose){
  graph_param_=GraphMapNavigatorParamPtr(new GraphMapNavigatorParam());
  graph_param_->GetParametersFromRos();
  regParam_=GraphFactory::CreateRegParam(registratorType);
//  cout<<"started reading reg par from ros"<<endl;
  regParam_->GetParametersFromRos();
  regParam_->sensor_pose=sensorPose;
  sensorPose_=sensorPose;
//  cout<<"finished reading reg par from ros"<<endl;
  mapParam_=GraphFactory::CreateMapParam(maptype);
//  cout<<"started reading map par from ros"<<endl;
  mapParam_->GetParametersFromRos();
//  cout<<"time to create graph inside fuser"<<endl;

  //Should init from the sensor pose for the node pose:
  Eigen::Affine3d Tsensor_pose = init_pose * sensorPose;
  graph_map_ =GraphMapNavigatorPtr(new GraphMapNavigator( Tsensor_pose, mapParam_,graph_param_));
  registrator_=GraphFactory::CreateRegistrationType(regParam_);
  use_keyframe_=graph_param_->use_keyframe;
  min_keyframe_dist_=graph_param_->min_keyframe_dist;
  min_keyframe_rot_deg_=graph_param_->min_keyframe_rot_deg;
  nr_frames_=0;
  pose_last_fuse_=init_pose;
  initialized_=true;
}
GraphMapFuser::GraphMapFuser(RegParamPtr regParam,  MapParamPtr mapParam, GraphMapNavigatorParamPtr graph_param, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose){
  mapParam_=mapParam;
  regParam_=regParam;
  graph_param_=graph_param;
  sensorPose_=sensorPose;
  Eigen::Affine3d Tinit=init_pose;

  graph_map_ =GraphMapNavigatorPtr(new GraphMapNavigator( Tinit,mapParam_,graph_param_));
  registrator_=GraphFactory::CreateRegistrationType(regParam_);

  nr_frames_=0;
  use_keyframe_=graph_param_->use_keyframe;
  min_keyframe_dist_=graph_param_->min_keyframe_dist;
  min_keyframe_rot_deg_=graph_param_->min_keyframe_rot_deg;
  pose_last_fuse_=init_pose;
  initialized_=true;
}

void GraphMapFuser::SaveGraphMap(const std::string &filename){
  cout<<"-----------------------------Saving---------------------------------\n"<<graph_map_->ToString()<<endl;
  cout<<"----------------------------------------------------------------------\nTo file path:"<<filename<<endl;
  std::ofstream ofs(filename);
  boost::archive::text_oarchive ar(ofs);
  ar << graph_map_;
  ofs.close();

}
void GraphMapFuser::SavePointCloud(const std::string &filename, bool original_points){ SaveGraphMapPLY(filename+"_.ply",graph_map_,original_points); }
void GraphMapFuser::SaveCurrentNodeAsJFF(const std::string &filename){
  cout<<"-----------------------------Saving JFF---------------------------------\n"<<graph_map_->ToString()<<endl;
  cout<<"----------------------------------------------------------------------\nTo file path:"<<filename<<endl;
  NDTMap * map=boost::dynamic_pointer_cast<NDTMapType> (graph_map_->GetCurrentNode()->GetMap())->GetNDTMap();
  std::string name=filename;
  map->writeToJFF(name.c_str());
}

Eigen::MatrixXd GraphMapFuser::PredictOdomUncertainty(const Eigen::Affine3d Tnow, bool prediction2d)
{
  Eigen::Affine3d Tmotion=GetPoseLastFuse().inverse()*Tnow;//difference since last fuse
  Eigen::MatrixXd cov6d = motion_model_3d_.getCovMatrix(Tmotion);
  //if(prediction2d)
  //  return ndt_generic::Cov6dTo3d(cov6d);
 //else
    return cov6d;
}
void GraphMapFuser::Visualize(bool enableVisualOutput,plotmarker marker){
  visualize_=enableVisualOutput;
  marker_=marker;
}


bool GraphMapFuser::FuseFrame(const Affine3d &Tnow, const Affine3d &Tmotion){
  if(fuse_no_motion_frames)
    return FuseNoMotionFrames(Tnow,Tmotion);
  else
    return KeyFrameBasedFuse(Tnow);
}

bool GraphMapFuser::KeyFrameBasedFuse(const Affine3d &Tnow ){

  bool ret = false;
  static int frame_idx = 0;
  if(frame_idx<=0){
    frame_idx++;
    return true;
  }
  else{
    frame_idx++;
    Affine3d diff=pose_last_fuse_.inverse()*Tnow;
    Eigen::Vector3d Tmotion_euler = diff.rotation().eulerAngles(0,1,2);
    ndt_generic::normalizeEulerAngles(Tmotion_euler);

    //cout<<"diff (trans[m]/rot[deg])=("<<diff.translation().norm()<<"/"<<diff.rotation().eulerAngles(0,1,2).norm()*180.0/M_PI<<") limit=("<<min_keyframe_dist_<<"/"<<min_keyframe_rot_deg_<<")"<<endl;
    if(use_keyframe_ ){
      if(diff.translation().norm()>min_keyframe_dist_ || Tmotion_euler.norm()>(min_keyframe_rot_deg_*M_PI/180.0)) {
        ret = true;
      }
      else {
        ret = false;
      }
    }
    else {
      ret = true;
    }
  }
  frame_idx++;
  if (ret == true && avoid_lidar_shadow) {
    if (frame_idx % 4 == 0) {// Avoid that the scanner covers the same region (one scan covers ~270 degrees angle and the same area is repeaded every 4rd frame)
      ret = false;
    }
    else {
      frame_idx = 0;
    }
  }
  return ret;
}

bool GraphMapFuser::FuseNoMotionFrames(const Affine3d &Tnow ,const Affine3d &Tmotion){
  const double no_mot_t=0.01, no_mot_r=M_PI/180.0; // 1cm /s resp. 1 deg/s
  const double min_dist=1.5;
  static unsigned int fused_frames=0;
  Vector3d euler=Tmotion.linear().eulerAngles(0,1,2);
  ndt_generic::normalizeEulerAngles(euler);

  Eigen::Affine3d diff;
  diff=pose_last_fuse_.inverse()*Tnow;
  if(Tmotion.translation().norm()*13<no_mot_t && euler.norm()*13.0<no_mot_r && diff.translation().norm()>min_dist)
  {
    if(fused_frames<100){
//      cout<<"FUSE frame="<<fused_frames<<", t_vel="<<Tmotion.translation().norm()/13.0<<" m/s , rot_vel="<<euler.norm()<<endl;
      fused_frames++;
      return true;
    }
  }
  else{
    fused_frames=0;
  }
  return false;
}

void GraphMapFuser::SetFuserOptions(bool save_merged_cloud){
  save_merged_cloud_=save_merged_cloud;
}

void GraphMapFuser::PlotMapType(){
  //NDTMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap());
  //GraphPlot::SendGlobalMapToRviz(curr_node->GetNDTMap(),1,graph_map_->GetCurrentNodePose());
  GraphPlot::PlotMap(graph_map_->GetCurrentNode()->GetMap(),-1,graph_map_->GetCurrentNodePose(),marker_);
  GraphPlot::PlotPoseGraph(graph_map_);
}

void GraphMapFuser::plotGTCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud){
  perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(0.4));
  // for(int i=0;i<cloud.size();i+=500){
  // cout<<cloud[i].x<<","<<cloud[i].y<<","<<cloud[i].z<<endl;
  // }
  ndlocal.guessSize(0,0,0,100,100,8);
  ndlocal.loadPointCloud(cloud,30.0);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
//  cout<<"CLOUD SIZE IS="<<cloud.size()<<endl;
//  cout<<"ndt cloud size="<<ndlocal.getAllCells().size()<<endl;
  NDTMap * ptrmap=&ndlocal;
  GraphPlot::SendLocalMapToRviz(ptrmap,1);
}

bool GraphMapFuser::ErrorStatus(string status){
  if(graph_map_!=NULL && registrator_!=NULL){
    return false;
  }
  else{
    status="No object instance found for graph or registrator";
    return true;
  }
}
//void GetCovarianceFromMotion(Matrix6d &cov,const Affine3d &Tm){
//}
std::string GraphMapFuser::ToString(){
  std::stringstream ss;
  ss<<"fuser:"<<endl<<"initialized:"<<initialized_<<endl;
  ss<<"visualize:"<<visualize_<<endl;
  ss<<"Key frame based update:"<<std::boolalpha<<use_keyframe_<<endl;
  if(use_keyframe_)
    ss<<"minimum keyframe distance(meter/deg):("<<min_keyframe_dist_<<","<<min_keyframe_rot_deg_<<")"<<endl;

  ss<<registrator_->ToString();
  ss<<graph_map_->ToString();
  return ss.str();
}
}
}
