#include "graph_map/octomap/octomap_map_type.h"
#include <boost/serialization/export.hpp>
//BOOST_CLASS_EXPORT(perception_oru::libgraphMap::NarfMapType)
namespace perception_oru{
namespace graph_map{

using namespace std;


OctomapMapType::OctomapMapType( MapParamPtr paramptr):MapType(paramptr){

  OctomapMapTypeParamPtr param = boost::dynamic_pointer_cast< OctomapMapTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Get parameters to this class from param
    cout<<"templateMapType: created templateMapType"<<endl;
    //occ=new pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> (0.1);
    tree_=new octomap::OcTree( 0.05 );


  }
  else
    cerr<<"templateMapType: Cannot create instance for \"templateMapType\""<<std::endl;
}
OctomapMapType::~OctomapMapType(){}
void OctomapMapType::update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple){//Mandatory, base method implemented as pure virtual}
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(cloud, cloud_xyz);
  update(Tsensor, cloud_xyz);

}


void OctomapMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud,bool simple){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.
  static bool first_time=true;
  if(!initialized_){
    initialized_=true;
    return;
  }


  for (auto p:cloud.points)
     tree_->updateNode( octomap::point3d(p.x, p.y, p.z), true );
  tree_->updateInnerOccupancy();

  cout<<"size: "<<tree_->calcNumNodes()<<endl;



/*
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud,*cloud2);
  cout<<"cloud size="<<cloud.size()<<endl;

  occ->setInputCloud(cloud2);
  occ->addPointsFromInputCloud();

  cout<<"leafs: "<<occ->getLeafCount()<<endl;
  ros::Time T1=ros::Time::now();
  cout<<"cloud size:"<<cloud.size()<<endl;

  /*
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
              << cloudB->points[newPointIdxVector[i]].y << " "
              << cloudB->points[newPointIdxVector[i]].z << std::endl;
   *
   * */
}
void OctomapMapType::GetPointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud){
  /* std::vector<int> newPointIdxVector;
    occ->getPointIndicesFromNewVoxels (newPointIdxVector);
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
    for (tree_it = occ->begin(); tree_it!=occ->end(); ++tree_it)
     {
      occ->
       Eigen::Vector3f voxel_min, voxel_max;
       octree.getVoxelBounds(tree_it, voxel_min, voxel_max);
       occ->getVoxelCentroids()

       pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
       pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
       pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
       displayCloud->points.push_back(pt);
 }*/
}

bool OctomapMapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){
  cout<<"please implement map compound for improved usage of submaps"<<endl;
  return true;//remove this
  if( OctomapMapTypePtr targetPtr=boost::dynamic_pointer_cast<OctomapMapType>(target) ){

    cout<<"\"CompoundMapsByRadius\" not overrided by template but not implemented"<<endl;
  }
}


void OctomapMapTypeParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
  nh.param<std::string>("Super_important_map_parameter",SuperImportantMapParameter,"parhaps not so important...");
}
}


}
