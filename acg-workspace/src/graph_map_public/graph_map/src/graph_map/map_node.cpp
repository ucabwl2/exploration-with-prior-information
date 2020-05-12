#include "graph_map/map_node.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::MapNode)
namespace perception_oru{
  namespace graph_map{
  Node::Node(){
    static unsigned int num_nodes=0;
    id_=num_nodes;
    num_nodes++;
    pose_=Eigen::Affine3d::Identity();
  }

      const Affine3d& Node::GetPose() const{
        return pose_;
      }
      Affine3d& Node::GetPose(){
        return pose_;
      }

  bool Node::operator ==(const Node& node_compare){
    if(id_==node_compare.id_ && pose_.isApprox(node_compare.pose_))
      return true;
    else return false;
  }

  bool Node::WithinRadius(const Affine3d &pose, const double &radius){

    double distance= Eigen::Vector3d(pose.translation()-pose_.translation()).norm();
    if(distance<radius)
      return true;
    else return false;
  }

  bool Node::DetectNewNode(unsigned int &node_id, NodePtr node){ //compare node with node_id, if different, node.GetId is assigned to node_id
    if(node->GetId()==node_id)
      return false;
    else{
      node_id=node->GetId();
      return true;
    }
  }


  MapNode::MapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam){

    ROS_DEBUG_STREAM( "New node at pose " << pose.matrix() );


    pose_=pose;
    map_=GraphFactory::CreateMapType(mapparam);
  }
  MapNode::MapNode(){
    map_=NULL;
  }
  Eigen::Affine3d MapNode::GetMapPose(bool obs_centroid){
    if(!obs_centroid)
      return GetPose();
    else
      return GetPose()*map_->GetObservationCentroid();
  }
  bool MapNode::WithinRadius(const Affine3d &pose, const double &radius, bool obs_centroid){
    double distance= Eigen::Vector3d(pose.translation()-GetMapPose(obs_centroid).translation()).norm();
    if(distance<radius)
      return true;
    else return false;
  }
  std::string MapNode::ToString(){
    std::stringstream ss;
    ss<<"MapNode:\ninitialized: "<<initialized_<<"\nid:"<<id_<<"\nPosition(x,y,z):"<<pose_.translation().transpose()<<std::endl;
    ss<<map_->ToString();
    return ss.str();
  }

  void MapNode::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud){
    map_->updateMap(Tnow,cloud);
    initialized_=true;
  }

  void MapNode::updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud){
    map_->updateMap(Tnow,cloud);
    initialized_=true;
  }
  ndt_generic::Affine3dSTLVek MapNode::GetObservationVector(bool local_frame){
    ndt_generic::Affine3dSTLVek obs_vec=map_->GetObservationVector();
    if(!local_frame){
      for(int i=0;i<obs_vec.size();i++)
        obs_vec[i]=GetPose()*obs_vec[i];
    }
    return obs_vec;
  }

  }
}
//#include <graph_map/map_node_impl.h>
