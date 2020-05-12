#include "graph_map/ndt/ndtd2d_reg_type.h"
namespace perception_oru{
namespace graph_map{


NDTD2DRegType::NDTD2DRegType(RegParamPtr paramptr):registrationType(paramptr){

  NDTD2DRegParamPtr param_ptr = boost::dynamic_pointer_cast< NDTD2DRegParam >(paramptr);//Should not be NULL
  if(param_ptr!=NULL){
    resolution_=param_ptr->resolution;
    resolutionLocalFactor_=param_ptr->resolution_local_factor;
    multires_=param_ptr->multires;
    SoftConstraints_=param_ptr->SoftConstraints;

    if(registration2d_){
      if(SoftConstraints_)
        matcher2D_=new NDTMatcherD2DSC_2D();
      else
        matcher2D_=new NDTMatcherD2D_2D();

      matcher2D_->ITR_MAX = param_ptr->matcher2D_ITR_MAX;
      matcher2D_->step_control=param_ptr->matcher2D_step_control;
      matcher2D_->n_neighbours=param_ptr->matcher2D_n_neighbours;
    }
    else{
      if(SoftConstraints_)
        matcher3D_=new NDTMatcherD2DSC(false);
      else
        matcher3D_=new NDTMatcherD2D();

      matcher3D_->ITR_MAX = param_ptr->matcher2D_ITR_MAX;
      matcher3D_->step_control=param_ptr->matcher2D_step_control;
      matcher3D_->n_neighbours=param_ptr->matcher2D_n_neighbours;
    }
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<std::endl;
}

NDTD2DRegType::~NDTD2DRegType(){}

bool NDTD2DRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov) {

  ///Create local map
  ///
  NDTMapPtr MapPtr = boost::dynamic_pointer_cast< NDTMapType >(maptype);
  NDTMap *globalMap=MapPtr->GetNDTMap();
  bool registration_status=true;
  if(multires_){
    for(int i=1;i>=0;i--){//should be i=2
      //  std::cout<<"REG resolution factor="<<i<<std::endl;
      perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution_*pow(2,i)));
      ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
      ndlocal.loadPointCloud(cloud,sensor_range);
      ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
      if(SoftConstraints_)
        std::cerr<<"Soft constraints disabled for multi-resolution registration"<<std::endl;
      if(registration2d_)
        registration_status= matcher2D_->match(*globalMap, ndlocal,Tnow,true);
      else
        registration_status= matcher3D_->match( *globalMap, ndlocal,Tnow,true);
    }
  }
  else if(!multires_){
    //std::cout<<"regular registration"<<std::endl;
    perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(resolution_*resolutionLocalFactor_));
    ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
    ndlocal.loadPointCloud(cloud,sensor_range);
    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    if(SoftConstraints_ && !registration2d_)
      registration_status= dynamic_cast<NDTMatcherD2DSC*>(matcher3D_)->match( *globalMap, ndlocal,Tnow,Tcov);
    else if(SoftConstraints_ && registration2d_){
      Eigen::Matrix3d Tcov2d=ndt_generic::Cov6dTo3d(Tcov);
      registration_status= dynamic_cast<NDTMatcherD2DSC_2D*>(matcher2D_)->match( *globalMap, ndlocal,Tnow,Tcov2d);
    }
    else if(!SoftConstraints_ && registration2d_)
      registration_status= matcher2D_->match(*globalMap, ndlocal,Tnow,true);
    else if(!SoftConstraints_ && !registration2d_)
      registration_status= matcher3D_->match( *globalMap, ndlocal,Tnow,true);

  }
  if(registration2d_)
    status_=matcher2D_->status_;
  else
    status_=matcher3D_->status_;
  return registration_status;
}
bool NDTD2DRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,Eigen::MatrixXd &Tcov) {
  cerr << "TODO: implement update for point type PointXYZIR - will convert to PointXYZ for now" << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(cloud, cloud_xyz);
  return Register(maptype, Tnow,cloud_xyz,Tcov);
}
bool NDTD2DRegType::RegisterMap2Map(MapTypePtr map_prev, MapTypePtr map_next, Eigen::Affine3d &Tdiff, double &match_score){

  std::cout<<"register diff: "<<Tdiff.translation().transpose()<<std::endl;
  if(map_prev==NULL || map_next==NULL){
    std::cout<<"map node is null"<<std::endl;
  }
  else if(!enable_registraiton)
    return true;

  NDTMap *prev,*next;
  Eigen::Affine3d Tinit=Tdiff;
  bool match_succesfull;
  prev=(boost::dynamic_pointer_cast<NDTMapType>(map_prev))->GetNDTMap();
  next=(boost::dynamic_pointer_cast<NDTMapType>(map_next))->GetNDTMap();
  if(registration2d_){
    match_succesfull=matcher2D_->match(*prev,*next,Tinit,true);
  }
  else if(!registration2d_){
    match_succesfull=matcher3D_->match( *prev,*next,Tinit,true);
  }
  if(match_succesfull){
    match_score=matcher3D_->finalscore;
    Tdiff=Tinit;
    std::cout<<"final score="<<match_score<<std::endl;
    std::cout<<"diff after reg-trans=\n"<<Tdiff.translation()<<std::endl;
    std::cout<<"diff after reg-rot=\n="<<Tdiff.linear()<<std::endl;
  }
  else
    std::cout<<"error registering maps"<<std::endl;


  return match_succesfull;
}
std::string NDTD2DRegType::ToString(){
  std::stringstream ss;
  ss<<registrationType::ToString();
  if(enable_registraiton){
    ss<<"NDT d2d registration type:"<<std::endl;
    ss<<"resolution :"<< resolution_<<std::endl;
    ss<<"resolutionLocalFactor :"<< resolutionLocalFactor_<<std::endl;
    ss<<"Soft constraints :"<< std::boolalpha<<SoftConstraints_<<std::endl;
    if(!registration2d_){
      ss<<"max opt itr :"<<matcher3D_->ITR_MAX<<std::endl;
      ss<<"opt step ctrl :"<<std::boolalpha<<matcher3D_->step_control<<std::endl;
      ss<<"d2d neighboors :"<<matcher3D_->n_neighbours<<std::endl;
    }
    else if(registration2d_){
      ss<<"max opt itr :"<<matcher2D_->ITR_MAX<<std::endl;
      ss<<"opt step ctrl :"<<std::boolalpha<<matcher2D_->step_control<<std::endl;
      ss<<"d2d neighboors :"<<matcher2D_->n_neighbours<<std::endl;
    }
  }
  else
    ss<<"Registration disabled:"<<std::endl;
  return ss.str();
}

/* ----------- Parameters ------------*/
NDTD2DRegParam::~NDTD2DRegParam(){}
NDTD2DRegParam::NDTD2DRegParam():registrationParameters(){}
void NDTD2DRegParam::GetParametersFromRos(){
  registrationParameters::GetParametersFromRos();
  std::cout<<"derived class read from ros"<<std::endl;
  ros::NodeHandle nh("~");//base class parameters
  nh.param("resolution",resolution,0.4);
  nh.param("resolutionLocalFactor",resolution_local_factor,1.0);
  nh.param("multires",multires,false);
}

}

}//end namespace

