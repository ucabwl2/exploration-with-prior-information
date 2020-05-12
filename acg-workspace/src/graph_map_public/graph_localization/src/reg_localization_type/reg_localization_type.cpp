#include "graph_localization/reg_localization_type/reg_localization_type.h"
namespace perception_oru{
namespace graph_localization{

RegLocalisationType::RegLocalisationType(LocalisationParamPtr param):LocalizationType(param){
  RegLocalisationParamPtr loc_param_ptr=boost::dynamic_pointer_cast<RegLocalisationParam>(param);
  RegParamPtr reg_par_ptr= loc_param_ptr->registration_parameters;

  if(reg_par_ptr==NULL || loc_param_ptr==NULL){
    if(reg_par_ptr==NULL)
      std::cerr<<"Registration parameters NULL for RegLocalisationType"<<endl;
    else if(loc_param_ptr==NULL)
      std::cerr<<"NULL parameters to RegLocalisationType"<<endl;
    exit(0);
  }
  keyframe_update_=loc_param_ptr->keyframe_update;
  min_keyframe_dist_=loc_param_ptr->min_keyframe_dist;
  min_keyframe_dist_rot_deg_=loc_param_ptr->min_keyframe_dist_rot_deg;
  graph_map_->SetMapSwitchMethod(param->switch_map_method);

  if(NDTD2DRegParamPtr ndt_reg_par=boost::dynamic_pointer_cast<NDTD2DRegParam>(reg_par_ptr)){
    ndt_reg_par->resolution=boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentNode()->GetMap())->GetResolution();
  }
  regptr_= GraphFactory::CreateRegistrationType(reg_par_ptr);
  counter=0;
  initialized_ = false;
}

bool RegLocalisationType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion){
  if(!initialized_){
    cerr<<"localisation not initialised"<<endl;
    return false;
  }
  counter++;
  Eigen::Affine3d Tinit=GetPose()*Tmotion; //prediction

  bool perform_registration;
  if(keyframe_update_){
    if(( Tinit.translation()-pose_last_update_.translation()).norm()>min_keyframe_dist_ || (Tinit*pose_last_update_.inverse()).rotation().eulerAngles(0,1,2).norm()>min_keyframe_dist_rot_deg_*M_PI/180.0 )//if movement
      perform_registration=true;
    else
      perform_registration=false;
  }
  else
    perform_registration=true;

  if(perform_registration){
    graph_map_->WorldToLocalMapFrame(Tinit);
    regptr_->visualize(visualize_,graph_map_->GetCurrentNodePose());
    Eigen::MatrixXd Tcov=Eigen::MatrixXd::Identity(6,6);
    bool successful=regptr_->RegisterScan(graph_map_->GetCurrentNode()->GetMap(),Tinit,cloud,Tcov);
    graph_map_->LocalToWorldMapFrame(Tinit);
    if(successful)
      pose_last_update_=Tinit;

    graph_map_->SwitchToClosestMapNode(Tinit*sensor_pose_,DBL_MAX,&cloud); //frame was changed
    SetPose(Tinit);
    return true;
  }

  SetPose(Tinit);
  return false;
}


bool RegLocalisationType::UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion){
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(cloud, cloud_xyz);
  return UpdateAndPredict(cloud_xyz, Tmotion);
}



std::string RegLocalisationType::ToString(){

  std::stringstream ss;
  ss<<LocalizationType::ToString();
  ss<<"scan-to-map reg localisation type:"<<endl;
  ss<<regptr_->ToString()<<endl;
  return ss.str();
}

void RegLocalisationType::InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance){ //Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
  SetPose(pose);
  pose_last_update_=pose;
  initialized_=true;
  bool new_current=graph_map_->SwitchToClosestMapNode(pose*sensor_pose_);
}
double RegLocalisationType::getDoubleTime()
{
  struct timeval time;
  gettimeofday(&time,NULL);
  return time.tv_sec + time.tv_usec * 1e-6;
}

void RegLocalisationType::NormaliseEulerAngles(Eigen::Vector3d &euler) {
  euler[0] = angles::normalize_angle(euler[0]);
  euler[1] = angles::normalize_angle(euler[1]);
  euler[2] = angles::normalize_angle(euler[2]);

  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] = -euler[1] + M_PI;
    euler[2] += M_PI;

    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}

RegLocalisationParam::RegLocalisationParam(){

  motion_model.push_back(0.05);
  motion_model.push_back(0.05);
  motion_model.push_back(0.02);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.02);

  motion_model.push_back(0.05);
  motion_model.push_back(0.1);
  motion_model.push_back(0.02);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.02);


  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.001);
  motion_model.push_back(0.001);
  motion_model.push_back(0.001);

  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);

  motion_model.push_back(0.01);
  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.01);

  motion_model.push_back(0.1);
  motion_model.push_back(0.01);
  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);

  /*motion_model_offset.push_back(0.00);
  motion_model_offset.push_back(0.002);
  motion_model_offset.push_back(0.0000001);//0.002
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.001);*/

  motion_model_offset.push_back(0.005);
  motion_model_offset.push_back(0.005);
  motion_model_offset.push_back(0.0000001);//0.002
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.003);
}
}
}
