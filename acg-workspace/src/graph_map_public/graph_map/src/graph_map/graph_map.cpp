#include "graph_map/graph_map.h"

namespace perception_oru{
namespace graph_map{
//    using namespace std;

GraphMap::GraphMap(const Affine3d &nodepose, const MapParamPtr &mapparam, const GraphMapParamPtr graphparam){
  factors_.clear();
  prevNode_=NULL;
  currentNode_=GraphFactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
  map_nodes_.push_back(currentNode_);
  mapparam_=mapparam;
}
GraphMap::GraphMap(){
  currentNode_=NULL;
  prevNode_=NULL;
  nodes_.clear();//Vector of all nodes in graph
  factors_.clear();
  map_nodes_.clear();
  mapparam_=NULL;//

}

MapNodePtr GraphMap::GetCurrentNode(){
  return currentNode_;
}
MapNodePtr GraphMap::GetPreviousNode(){
  return prevNode_;
}
void GraphMap::RemoveNode(NodePtr node){
  for(int i=0;i<nodes_.size();i++){
    if(nodes_[i]==node)
      nodes_.erase(nodes_.begin()+i);
  }
  for(int i=0;i<map_nodes_.size();i++){
    if(map_nodes_[i]==node)
      map_nodes_.erase(map_nodes_.begin()+i);

  }
}

void GraphMap::AddMapNode(const Affine3d &diff, const Matrix6d &cov){ //Add node with link uncertainty

  Affine3d newNodePose=Affine3d::Identity();
//	std::cout << "Add map node1 " << std::endl;
  newNodePose= GetCurrentNodePose()*diff;
//	std::cout << "Add map node2 " << std::endl;
  MapNodePtr newNode=GraphFactory::CreateMapNode(newNodePose,mapparam_);
//  std::cout << "Add map node3 " << std::endl;
  FactorPtr sd=GraphFactory::CreateMapNodeFactor(currentNode_,newNode,diff,cov);
//  std::cout << "Add map node4 " << std::endl;
  factors_.push_back(sd);//Add connection between current and new node with link diff and covariance
//  std::cout << "Add map node5 " << std::endl;
  nodes_.push_back(newNode);
//  std::cout << "Add map node6 " << std::endl;
  map_nodes_.push_back(newNode);
//	std::cout << "Add map node6.5 " << std::endl;
  prevNode_=currentNode_;
//  std::cout << "Add map node7 " << std::endl;
  currentNode_=newNode;
//  std::cout << "Add map node8 " << std::endl;
}
void GraphMap::UpdateLink(MapNodePtr prev, MapNodePtr next,const Affine3d &Tdiff, const Matrix6d &cov){

  next->SetPose(prev->GetPose()*Tdiff);
  //cout<<"find factor between node:"<<prev->GetId()<<" and "<<next->GetId()<<endl;
  for(int i=0;i<factors_.size();i++){
    if(factors_[i]->Connects(prev) && factors_[i]->Connects(next) ){
      factors_[i]->UpdateFactor(Tdiff);
      std::cout<<"Update Link: "<<i<<std::endl;
    }
  }
}
std::string GraphMap::ToString(){
  std::stringstream ss;
  ss<<std::endl<<"GraphMap:"<<std::endl;
  ss <<"Graph size="<<nodes_.size()<<std::endl;
  for(int i=0;i<nodes_.size();i++){
    if(i==0)
      ss <<"Node positions:"<<std::endl;

    NodePtr ptr=nodes_[i];
    Eigen::Vector3d position=ptr->GetPose().translation();
    ss<<"node "<<i<<" (x,y,z):("<<position(0)<<","<<position(1)<<","<<position(2)<<")"<<std::endl;
  }
  if(currentNode_!=NULL)
    ss<<"Detailed info node 0:"<<std::endl<<currentNode_->ToString();

  return ss.str();
}

    const Affine3d& GraphMap::GetNodePose(int nodeNr) const {
      if(nodeNr<Size())
        return nodes_[nodeNr]->GetPose();
    }

    Affine3d& GraphMap::GetNodePose(int nodeNr) {
      if(nodeNr<Size())
        return nodes_[nodeNr]->GetPose();
    }
Affine3d GraphMap::GetCurrentNodePose( bool use_obs_centroid){
  return currentNode_->GetMapPose(use_obs_centroid);
}
Affine3d GraphMap::GetPreviousNodePose(bool use_obs_centroid){
  return prevNode_->GetMapPose(use_obs_centroid);
}
MapNodePtr GraphMap::GetNode(unsigned int node_id){
  if(node_id < map_nodes_.size()){
    for(int i=0;i<map_nodes_.size();i++){
      if(node_id==map_nodes_[i]->GetId())
        return map_nodes_[i];
    }
  }
  return NULL;
}

void GraphMap::GetCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, bool original_points){
  cloud.clear();
  for(int i=0;i<map_nodes_.size();i++){
    pcl::PointCloud<pcl::PointXYZ> curr_node_cloud;
    map_nodes_[i]->GetMap()->GetCloud(curr_node_cloud,original_points);
    Eigen::Affine3d cloud_offset=map_nodes_[i]->GetPose();
    transformPointCloudInPlace(cloud_offset,curr_node_cloud);
    cloud+=curr_node_cloud;
  }


  if(original_points){
    ROS_DEBUG_STREAM("original points, filtering");
    pcl::PCLPointCloud2::Ptr cloud_copy (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_downsampled (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(cloud,*cloud_copy);//  toPCLPointCloud2

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_copy);
    sor.setMinimumPointsNumberPerVoxel(3);
    sor.setLeafSize (0.04f, 0.04f, 0.04f);
    sor.filter (*cloud_downsampled);
    cloud.clear();

    pcl::fromPCLPointCloud2(*cloud_downsampled,cloud);
  }

}

void SaveGraphMapPLY(const std::string &filename,GraphMapPtr graph_map, bool original_points){

  pcl::PointCloud<pcl::PointXYZ> cloud;
  graph_map->GetCloud(cloud,original_points);
  ROS_DEBUG_STREAM("got could of size"<<cloud.size());
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
}
void GraphMap::AddFactor(MapNodePtr prev, MapNodePtr next, const Eigen::Affine3d &Tdiff, const Matrix6d &cov){
  factors_.push_back(GraphFactory::CreateMapNodeFactor(prev,next,Tdiff,cov));
}

GraphMapParam::GraphMapParam(){}
void GraphMapParam::GetParametersFromRos(){
}
}
}
