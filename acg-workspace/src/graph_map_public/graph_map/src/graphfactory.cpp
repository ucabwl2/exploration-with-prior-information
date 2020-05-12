#include "graph_map/graphfactory.h"
#include "graph_map/factor.h"
#include "graph_map/map_type.h"
#include "graph_map/ndt/ndt_map_param.h"
#include "graph_map/ndt/ndt_map_type.h"
#include "graph_map/map_node.h"
#include "graph_map/graph_map.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/reg_type.h"
#include "graph_map/ndt_dl/ndtdl_reg_type.h"
#include "graph_map/ndt/ndtd2d_reg_type.h"
#include "Eigen/Geometry"
#include "graph_map/template/template_map_type.h"
#include "graph_map/template/template_reg_type.h"
#include "graph_map/narf/narf_map_type.h"
#include "graph_map/narf/narf_reg_type.h"
#include "graph_map/octomap/octomap_map_type.h"


namespace perception_oru{
namespace graph_map{

bool GraphFactory::disable_output_=false;
//!
//! \brief GraphFactory::CreateMapParam Creates parameter type based on string input
//! \param mapname
//! \return
//!
//!
MapParamPtr GraphFactory::CreateMapParam(std::string mapname){
  if(mapname.compare(ndt_map_type_name)==0){
    if(!disable_output_) {
        ROS_DEBUG_STREAM("Graphfactory: Created parameters for map type: \"" << ndt_map_type_name << "\"");
    }
    NDTMapParamPtr paramPtr(new NDTMapParam());
    return paramPtr;
  }
  else if(mapname.compare(ndtdl_map_type_name)==0){

    if(!disable_output_) {
        ROS_DEBUG_STREAM("Graphfactory: Created parameters for map type: \"" << ndtdl_map_type_name << "\"");
    }
    NDTDLMapParamPtr paramPtr(new NDTDLMapParam());
    return paramPtr;
  }
  else if(mapname.compare(template_map_name)==0){
    TemplateMapParamPtr templatePar(new TemplateMapParam());
    if(!disable_output_) {
        ROS_ERROR_STREAM("Graphfactory:  Template, no map parameter instance created");
    }
    return templatePar;
  }
  else if(mapname.compare(narf_map_name)==0){
    NarfMapParamPtr narf_map_par(new NarfMapParam());
    if(!disable_output_) {
        ROS_ERROR_STREAM("Graphfactory:  narf map parameters created");
    }
    return narf_map_par;
  }
  else if(mapname.compare(octomap_map_name)==0){
    OctomapMapTypeParamPtr octomap_map_par(new OctomapMapTypeParam());
    if(!disable_output_)
        ROS_ERROR_STREAM("Graphfactory:  octomap map parameters created");
    return octomap_map_par;
  }
  else{
    if(!disable_output_)
        ROS_ERROR_STREAM("No map type exists with name: \""<<mapname<<"\"");
    return NULL;
  }
}

MapTypePtr GraphFactory::CreateMapType(const MapParamPtr & mapparam){
  if(  NDTMapParamPtr ndt2MapParam = boost::dynamic_pointer_cast< NDTMapParam >(mapparam) ){ //perform typecast and check if not null conversion

    if(!disable_output_)
        ROS_DEBUG_STREAM("Graphfactory: Created map of type: \""<<ndt_map_type_name<<"\"");
    return  MapTypePtr(new NDTMapType(ndt2MapParam));

  }
  else if(  NDTDLMapParamPtr ndtdlMapParam = boost::dynamic_pointer_cast< NDTDLMapParam >(mapparam) ){ //perform typecast and check if not null conversion
    if(!disable_output_)
        ROS_DEBUG_STREAM("Graphfactory: Created map of type: \""<<ndtdl_map_type_name<<"\"");
    return  MapTypePtr(new NDTDLMapType(ndtdlMapParam));
  }
  else if(TemplateMapParamPtr template_par=boost::dynamic_pointer_cast<TemplateMapParam>(mapparam)){
    if(!disable_output_)
        ROS_DEBUG_STREAM("Graphfactory: no map exists for \"template\"");
    return NULL;
  }
  else if(  NarfMapParamPtr narfMapParam = boost::dynamic_pointer_cast< NarfMapParam >(mapparam) ){ //perform typecast and check if not null conversion
    if(!disable_output_)
        ROS_DEBUG_STREAM("Graphfactory: Created map of type: \""<<narf_map_name<<"\"");
    return  MapTypePtr(new NarfMapType(narfMapParam));
  }
  else if(  OctomapMapTypeParamPtr octomap_param = boost::dynamic_pointer_cast< OctomapMapTypeParam >(mapparam) ){ //perform typecast and check if not null conversion
    if(!disable_output_)
        ROS_DEBUG_STREAM("Graphfactory: Created map of type: \""<<narf_map_name<<"\"");
    return  MapTypePtr(new OctomapMapType(octomap_param));
  }
  else{
    if(!disable_output_)
        ROS_DEBUG_STREAM("Graphfactory: No map type exists for map parameters");

    return NULL;
  }

}

GraphMapPtr GraphFactory::CreateGraph(const Eigen::Affine3d &nodepose, MapParamPtr &mapparam, GraphMapParamPtr graphparam){

  if(mapparam!=NULL){
    if(!disable_output_) {
        ROS_DEBUG_STREAM("Graphfactory: Creating graph");
    }
    GraphMapPtr graphPtr=GraphMapPtr(new GraphMapNavigator(nodepose,mapparam,graphparam));
    return graphPtr;
  }
  else{
    if(!disable_output_) {
        ROS_DEBUG_STREAM("Graphfactory: parameter NULL to graph");
    }
    return NULL;
  }
}


MapNodePtr GraphFactory::CreateMapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam){//Create a node

  if(!disable_output_) {
      ROS_DEBUG_STREAM("Graphfactory: Creating map node");
  }
  return boost::shared_ptr<MapNode>(new MapNode(pose,mapparam));
}


RegTypePtr GraphFactory::CreateRegistrationType(RegParamPtr regparam){


  if(NDTD2DRegParamPtr ndt_reg_ptr=boost::dynamic_pointer_cast<NDTD2DRegParam>(regparam)){
    if(!disable_output_) {
        ROS_DEBUG_STREAM("Graphfactory: created registration type:" << ndt_d2d_reg_type_name);
    }
    return NDTD2DRegTypePtr(new NDTD2DRegType(ndt_reg_ptr));
  }

  else if(  NDTDLRegTypeParamPtr ndt_reg_ptr=boost::dynamic_pointer_cast<NDTDLRegTypeParam>(regparam) ){ //perform typecast and check if not null conversion
    if(!disable_output_) {
        ROS_DEBUG_STREAM("Graphfactory: created registration type:" << ndt_dl_reg_type_name);
    }
    return NDTDLRegTypePtr(new NDTDLRegType(ndt_reg_ptr));
  }

  if(!disable_output_) {
      std::cerr << "Failed to create object of registration type" << std::endl;
  }
  return NULL;
}
RegParamPtr GraphFactory::CreateRegParam(std::string regType){
  if(regType.compare(ndt_d2d_reg_type_name)==0){
    if(!disable_output_)
      std::cout<<"Graphfactory: Creating parameters for registration type: \""<<ndt_d2d_reg_type_name<<"\""<<std::endl;
    return NDTD2DRegParamPtr(new NDTD2DRegParam());
  }
  else if(regType.compare(ndt_dl_reg_type_name)==0){
    if(!disable_output_)
        std::cout<<"Graphfactory: Creating parameters for registration type: \""<<ndtdl_map_type_name<<"\""<<std::endl;
    return NDTDLRegTypeParamPtr(new NDTDLRegTypeParam());
  }
  else{
    if(!disable_output_)
        std::cerr<<"No registration type with name: \""<<regType<<"\""<<std::endl;

    return NULL;
  }
}

FactorPtr GraphFactory::CreateMapNodeFactor(MapNodePtr prevMapPose, MapNodePtr nextMapPose, const Eigen::Affine3d &diff, const Matrix6d &covar){
  ROS_DEBUG_STREAM( "creating factor" );
  FactorPtr factorptr=boost::shared_ptr<factor>(new factor(prevMapPose,nextMapPose,diff,covar));
  ROS_DEBUG_STREAM( "creating factor done" );
  return factorptr;
}

}
}
