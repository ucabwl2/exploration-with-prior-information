#include "graph_map/template/template_map_type.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::TemplateMapType)
namespace perception_oru{
namespace graph_map{

//using namespace std;


TemplateMapType::TemplateMapType( MapParamPtr paramptr) : MapType(paramptr){
  TemplateMapParamPtr param = boost::dynamic_pointer_cast< TemplateMapParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Get parameters to this class from param
    std::cout<<"templateMapType: created templateMapType"<<std::endl;
  }
  else
    std::cerr<<"templateMapType: Cannot create instance for \"templateMapType\""<<std::endl;
}
TemplateMapType::~TemplateMapType(){}

void TemplateMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

  if(initialized_){
    //Initialize map
  }else{
    //Update map
    initialized_ = true;
  }
}

bool TemplateMapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){
  std::cout<<"please implement map compound for improved usage of submaps"<<std::endl;
  return true;//remove this
  if( TemplateMapTypePtr targetPtr=boost::dynamic_pointer_cast<TemplateMapType>(target) ){

    std::cout<<"\"CompoundMapsByRadius\" not overrided by template but not implemented"<<std::endl;
  }
}





TemplateMapParam::TemplateMapParam(){}


void TemplateMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  std::cout<<"reading parameters from ros inside GetRosParamNDT2D"<<std::endl;
  nh.param<std::string>("Super_important_map_parameter",SuperImportantMapParameter,"parhaps not so important...");
}
}


}
