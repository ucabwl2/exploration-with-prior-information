#include "graph_map/ndt_dl/ndtdl_reg_type.h"

namespace perception_oru{
namespace graph_map{

/* ----------- Parameters ------------*/
NDTDLRegTypeParam::~NDTDLRegTypeParam(){}
NDTDLRegTypeParam::NDTDLRegTypeParam():registrationParameters(){}
void NDTDLRegTypeParam::GetParametersFromRos(){
  registrationParameters::GetParametersFromRos();
  ros::NodeHandle nh("~");//base class parameters
  nh.param<std::string>("super_important_parameter",super_important_parameter_,"default string");
}


NDTDLRegType::NDTDLRegType(RegParamPtr paramptr):registrationType(paramptr){

  NDTDLRegTypeParamPtr param = boost::dynamic_pointer_cast< NDTDLRegTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Transfer all parameters from param to this class
    std::cout<<"Created registration type for template"<<std::endl;
  }
  else
    std::cerr<<"ndtd2d registrator has NULL parameters"<<std::endl;
}

NDTDLRegType::~NDTDLRegType(){}

template<class PointT>
bool NDTDLRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov) {

  std::cout<<"registration is disabled until it is implemented for map of type: "<<maptype->GetMapName()<<std::endl;
  return true;//Remove when registration has been implemented

  if(!enable_registraiton||!maptype->Initialized()){
    std::cout<<"Registration disabled - motion based on odometry"<<std::endl;

    return false;
  }
  else{
    NDTDLMapPtr MapPtr = boost::dynamic_pointer_cast< NDTDLMapType >(maptype);
    //Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
  }

}
}







}//end namespace

