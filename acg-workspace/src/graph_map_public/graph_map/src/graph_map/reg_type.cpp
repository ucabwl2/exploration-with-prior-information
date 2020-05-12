#include "graph_map/reg_type.h"
namespace perception_oru{
namespace graph_map{

bool registrationType::RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud){
  Eigen::MatrixXd cov=Eigen::MatrixXd::Identity(6,6);
  return RegisterScan(maptype,Tnow,cloud,cov);
}

bool registrationType::RegisterScan(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov){
  if(!enable_registraiton || !maptype->Initialized()){
    //std::cout<<"Registration disabled"<<std::endl;
    return true;
  }

  Eigen::Affine3d Tinit = use_initial_guess ? Tnow : Eigen::Affine3d::Identity();
  bool registration_return=Register(maptype,Tinit,cloud,Tcov);//please overwrite this function or use one of the existing registration methods...
  Eigen::Affine3d diff= Tnow.inverse()*Tinit;
  Eigen::Vector3d euler=diff.rotation().eulerAngles(0,1,2);
  ndt_generic::normalizeEulerAngles(euler);
  double angle_rot_norm=euler.norm();

  if(registration_return==false){ //unsuccesfull registration
    if(status_==MAX_ITR){
      std::cout<<"Registration max iterations"<<std::endl;
      Tnow=Tinit;
    }
    else
      std::cerr<<"Registration Internal failure"<<std::endl;
    failed_registrations++;
    return false;
  }
  else{//registration succesfull
    //Kstd::cout<<"registraiotn diff="<<diff.translation()<<std::endl;
    if(check_consistency && diff.translation().norm() > max_translation_norm ){
      std::cerr<<"Registration failure: Translation far from prediction, "<<diff.translation().norm()<<"m  >  "<<max_translation_norm<<std::endl;
      failed_registrations++;
      return false;
    }
    else if(check_consistency &&(angle_rot_norm> max_rotation_norm) ){
      std::cerr<<"Registration failure: Orientation too far from prediction, "<<angle_rot_norm<<"rad  >  "<<max_rotation_norm<<"rad"<<std::endl;
      failed_registrations++;
      return false;
    }
    else{
      Tnow=Tinit;//All well, translation and rotation feasible
      std::cout<<"Registration success"<<std::endl;
      succesfull_registrations++;
      return true;
    }
  }

}
bool registrationType::RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud){
  Eigen::MatrixXd cov=Eigen::MatrixXd::Identity(6,6);
  return RegisterScan(maptype,Tnow,cloud,cov);
}
bool registrationType::RegisterScan(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, Eigen::MatrixXd  &Tcov){
  if(!enable_registraiton){
    std::cout<<"Registration disabled"<<std::endl;
    return true;
  }

  Eigen::Affine3d Tinit=Tnow;
  bool registration_return=Register(maptype,Tinit,cloud,Tcov);//please overwrite this function or use one of the existing registration methods...

  Eigen::Affine3d diff= Tinit*Tnow.inverse();
  if(registration_return==false){ //unsuccesfull registration
    std::cerr<<"Registration Internal failure"<<std::endl;
    failed_registrations++;
    return false;
  }
  else{//registration succesfull
    if(check_consistency && diff.translation().norm() > max_translation_norm ){
      std::cerr<<"Registration failure: Translation far from prediction, "<<diff.translation().norm()<<"m  >  "<<max_translation_norm<<std::endl;
      failed_registrations++;
      return false;
    }
    else if(check_consistency &&(diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) ){
      std::cerr<<"Registration failure: Orientation too far from prediction, (also euler not normalized!!)"<<diff.rotation().eulerAngles(0,1,2).norm()<<"rad  >  "<<max_rotation_norm<<"rad"<<std::endl;
      failed_registrations++;
      return false;
    }
    else{
      Tnow=Tinit;//All well, translation and rotation feasible
      std::cout<<"Registration succesfull"<<std::endl;
      succesfull_registrations++;
      return true;
    }
  }
}

/* -------Registration type---------------- */
registrationType::registrationType(RegParamPtr regparam){
  if(regparam!=NULL){
    sensorPose_=regparam->sensor_pose;
    std::cout<<"created registration type"<<std::endl;
    enable_registraiton = regparam->enable_registration;
    registration2d_     = regparam->registration2d;
    check_consistency   = regparam->check_consistency;
    max_translation_norm = regparam->max_translation_norm;
    max_rotation_norm    = regparam->max_rotation_norm;
    rotation_registration_delta=regparam->rotation_registration_delta;
    sensor_range        =regparam->sensor_range;
    map_size_z           =regparam->map_size_z;
    use_initial_guess    =regparam->use_initial_guess;
    failed_registrations=0;
    succesfull_registrations=0;
    std::cout<<"sucessfully applied registration parameters"<<std::endl;
  }
  else
    std::cerr<<"Registration parameters cannot be applied to registrator as parameter object does not exist"<<std::endl;
}
registrationType::~registrationType(){}
std::string registrationType::ToString(){
  std::stringstream ss;
  ss<<std::endl<<"registration type:"<<std::endl;
  ss<<"enableRegistration: "<<std::boolalpha<<enable_registraiton<<std::endl;
  if(enable_registraiton){
    ss<<"registration limited to 2d: "<<std::boolalpha<<registration2d_<<std::endl;
    ss<<"Check consistency: "<<std::boolalpha<<check_consistency<<std::endl;
    ss<<"max registration distances(translation,rotation): ("<<max_translation_norm<<","<<max_rotation_norm<<")"<<std::endl;
    ss<<"Maximum sensor range: "<<sensor_range<<std::endl;
    ss<<"Map size z: "<<map_size_z<<std::endl;
    ss<<"sensor position offset: (x,y,z): ("<<sensorPose_.translation().transpose()(0)<<","<<sensorPose_.translation().transpose()(1)<<","<<sensorPose_.translation().transpose()(2)<<")"<<std::endl;
  }
  return ss.str();
}

/* -------Parameters---------------- */
registrationParameters::registrationParameters(){}
registrationParameters::~registrationParameters(){}
void registrationParameters::GetParametersFromRos(){
  std::cout<<"base class read ros parameters"<<std::endl;
  bool render_GT_map;
  ros::NodeHandle nh("~");//base class parameters
  std::cout<<"reading base class registration parameters"<<std::endl;
  nh.param("enable_registration",enable_registration,true);
  nh.param("registration_2D",registration2d,false);
  nh.param("check_consistency",check_consistency,true);
  nh.param("sensor_range",sensor_range,20.0);
  nh.param("size_z_meters",map_size_z,0.8);
  nh.param("max_translation_norm",max_translation_norm,0.4);
  nh.param("max_rotation_norm",max_rotation_norm,M_PI/4);
  nh.param("renderGTmap",render_GT_map,false);
  nh.param("do_soft_constraints",do_soft_constraints,false);
  /*
  nh.param<double>("motion_params_Cd", motion_model_2d_.params.Cd, 0.005);
  nh.param<double>("motion_params_Ct", motion_model_2d_.params.Ct, 0.01);
  nh.param<double>("motion_params_Dd", motion_model_2d_.params.Dd, 0.001);
  nh.param<double>("motion_params_Dt", motion_model_2d_.params.Dt, 0.01);
  nh.param<double>("motion_params_Td", motion_model_2d_.params.Td, 0.001);
  nh.param<double>("motion_params_Tt", motion_model_2d_.params.Tt, 0.005);
*/
  if(render_GT_map)
    enable_registration=false;
}

}
}

