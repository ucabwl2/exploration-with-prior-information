#include "graph_map/template/template_reg_type.h"
namespace perception_oru{
namespace graph_map{


TemplateRegType::TemplateRegType(RegParamPtr paramptr):registrationType(paramptr){

  TemplateRegTypeParamPtr param = boost::dynamic_pointer_cast< TemplateRegTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Transfer all parameters from param to this class
    std::cout<<"Created registration type for template"<<std::endl;
  }
  else
    std::cerr<<"ndtd2d registrator has NULL parameters"<<std::endl;
}

TemplateRegType::~TemplateRegType(){}

bool TemplateRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov) {

  if(!enable_registraiton||!maptype->Initialized()){
    std::cout<<"Registration disabled - motion based on odometry"<<std::endl;

    return false;
  }
  else{
    TemplateMapTypePtr MapPtr = boost::dynamic_pointer_cast< TemplateMapType >(maptype);
    //Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
    std::cout<<"please fill in code for registration- uintil then, registration is disabled"<<std::endl;
  }
  return false;//remove this when registration code has been implemented

}






/* ----------- Parameters ------------*/
TemplateRegTypeParam::~TemplateRegTypeParam(){}
TemplateRegTypeParam::TemplateRegTypeParam():registrationParameters(){
}
void TemplateRegTypeParam::GetParametersFromRos(){
  registrationParameters::GetParametersFromRos();
  ros::NodeHandle nh("~");//base class parameters
  nh.param<std::string>("super_important_parameter",super_important_parameter_,"default string");

}

}

}//end namespace

