#include "graph_map/ndt/ndt_map_param.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTMapParam)

namespace perception_oru{
namespace graph_map{
//    using namespace std;



void NDTMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
	std::cout<<"reading parameters from ros inside GetRosParamNDT2D"<<std::endl;
  nh.param("resolution",resolution_,1.0);
  //nh.param("laser_variance_z",varz,resolution/4);
}




}
}
