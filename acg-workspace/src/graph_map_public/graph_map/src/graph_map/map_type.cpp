#include "graph_map/map_type.h"
namespace perception_oru{
namespace graph_map{


//:octree(0.02)
MapType::MapType(){
  sizex_= 0;
  sizey_= 0;
  sizez_= 0;
  max_range_=100;
  min_range_=0.6;
  initialized_=false;
  enable_mapping_=true;
  store_points_=false;
  mapName_="";
}
//:octree(0.02)
MapType::MapType(MapParamPtr param){
  initialized_=false;
  enable_mapping_=param->enable_mapping_;
  sizex_= param->sizex_;
  sizey_= param->sizey_;
  sizez_= param->sizez_;
  max_range_=param->max_range_;
  min_range_=param->min_range_;
  store_points_=param->store_points;
  mapName_="";
}
void MapType::GetCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, bool original_points){
  for(int i=0;i<cloud.size();i++)
    cloud+=input_cloud_[i];
}
void MapType::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple){
  if(enable_mapping_){
    stamp= cloud.header.stamp;
    update(Tnow,cloud,simple);
    pose_est_.push_back(Tnow);
    mean_obs_location_.translation()=(Tnow.translation()+observations_*mean_obs_location_.translation())/(observations_+1.0);
    observations_++;
    if(store_points_)
      input_cloud_.push_back(cloud);
    //const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ptr=boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(&cloud);
    //octree.setInputCloud(ptr);
    //octree.addPointsFromInputCloud();
  }
  else
    std::cout<<"Mapping disabled"<<std::endl;

}
void MapType::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple){
  if(enable_mapping_){
    stamp= cloud.header.stamp;
    update(Tnow,cloud,simple);

    pose_est_.push_back(Tnow);
    mean_obs_location_.translation()=(Tnow.translation()+observations_*mean_obs_location_.translation())/(observations_+1.0);
  }
  else
    std::cout<<"Mapping disabled"<<std::endl;
}

bool MapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){
  std::cout<<"Compunding map not possible in base class"<<std::endl;
  return false;
}
std::string MapType::ToString(){
  std::stringstream ss;
  ss<<"MapType\nInitialized: "<<std::boolalpha<<initialized_<<std::endl;
  ss<<"Map Name: "<<mapName_<<std::endl;
  ss<<"Mapping enabled: "<<std::boolalpha<<enable_mapping_<<std::endl;
  ss<<"Size(x,y,z): ("<< sizex_<<","<<sizey_<<","<<sizez_<<")"<<std::endl;
  ss<<"Sensor range(min/max): ("<<min_range_<<","<<max_range_<<")"<<std::endl;
  return ss.str();
}

MapParam::~MapParam(){

}


void MapParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  nh.param("max_range",max_range_,100.);
  nh.param("min_range",min_range_,0.6);
  nh.param("size_x_meters",sizex_,.50);
  nh.param("size_y_meters",sizey_,.50);
  nh.param("size_z_meters",sizez_,.10);
  nh.param("enable_mapping",enable_mapping_,true);
  std::cout<<"read mapType parameters from ros"<<std::endl;
  std::cout<<ToString()<<std::endl;
}
std::string MapParam::ToString(){
  std::stringstream ss;
  ss<<"Base map parameters:"<<std::endl;
  ss<<"Range(max/min)=("<<max_range_<<"/"<<min_range_<<std::endl;
  ss<<"size(x,y,z)=("<<sizex_<<","<<sizey_<<","<<sizez_<<")";
  return ss.str();
}
}
}
