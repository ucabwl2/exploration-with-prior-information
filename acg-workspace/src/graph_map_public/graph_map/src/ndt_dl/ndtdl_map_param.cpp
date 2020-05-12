#include "graph_map/ndt_dl/ndtdl_map_param.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTDLMapParam)
namespace perception_oru{
namespace graph_map{
//    using namespace std;


void NDTDLMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
	std::cout<<"reading parameters from ros inside NDTDLMapParam::GetRosParametersFromRos()"<<std::endl;
  nh.param<std::string>("Super_important_map_parameter",SuperImportantMapParameter,"parhaps not so important...");
  nh.param("resolution",resolution,1.0);
}


}


}
