#include "graph_map/narf/narf_reg_type.h"

namespace perception_oru{
namespace graph_map{


NarfRegType::NarfRegType(RegParamPtr paramptr):registrationType(paramptr){

  NarfRegTypeParamPtr param = boost::dynamic_pointer_cast< NarfRegTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Transfer all parameters from param to this class
    cout<<"Created registration type for template"<<endl;
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<endl;
}

NarfRegType::~NarfRegType(){}

bool NarfRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov) {

  if(!enable_registraiton||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;

    return false;
  }
  else{
    NarfMapTypePtr MapPtr = boost::dynamic_pointer_cast< NarfMapType >(maptype);
    //Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
    cout<<"please fill in code for registration- uintil then, registration is disabled"<<endl;
  }
  return false;//remove this when registration code has been implemented

}




/* ----------- Parameters ------------*/
NarfRegTypeParam::~NarfRegTypeParam(){}
NarfRegTypeParam::NarfRegTypeParam():registrationParameters(){
}
void NarfRegTypeParam::GetParametersFromRos(){
  registrationParameters::GetParametersFromRos();
  ros::NodeHandle nh("~");//base class parameters
  nh.param<std::string>("super_important_parameter",super_important_parameter_,"default string");

}

}

}//end namespace

