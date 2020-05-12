#ifndef GRAPH_MAP_H
#define GRAPH_MAP_H
#include "graphfactory.h"
#include "graph_map/factor.h"
#include "graph_map/map_type.h"
#include "graph_map/map_node.h"
#include "stdio.h"
#include <iostream>
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include <stdint.h>
#include <Eigen/StdVector>
#include "graph_map/graph_plot.h"
#include "ndt_map/ndt_map.h"
#include "ndt/ndt_map_type.h"
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "boost/serialization/serialization.hpp"
#include "boost/serialization/vector.hpp"
#include "graph_map/voxelgrid.h"
#include "octomap/octomap.h"
#include "pcl/io/ply_io.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
namespace perception_oru{
namespace graph_map{
using namespace perception_oru;

typedef std::vector<MapNodePtr>::iterator mapNodeItr;

	typedef std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > EigenAffineVector;
	typedef std::vector<NodePtr,Eigen::aligned_allocator<NodePtr> > NodePtrVector;

class GraphMap{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraphMap();

  GraphMap(const Eigen::Affine3d &nodepose, const MapParamPtr &mapparam, const GraphMapParamPtr graphparam);

  virtual void AddMapNode(const Eigen::Affine3d &diff, const Matrix6d &cov=unit_covar);

  virtual void RemoveNode(NodePtr node);

  void AddFactor(MapNodePtr prev, MapNodePtr next, const Affine3d &Tdiff, const Matrix6d &cov);

  void UpdateLink(MapNodePtr prev, MapNodePtr next,const Affine3d &Tdiff, const Matrix6d &cov=unit_covar);

  MapNodePtr GetCurrentNode();

  MapNodePtr GetPreviousNode();

  uint32_t Size() const {return nodes_.size();}

  MapNodePtr GetNode(unsigned int node_id); //as occuring in the node list

	const Eigen::Affine3d& GetNodePose(int nodeNr) const;
	Eigen::Affine3d& GetNodePose(int nodeNr);

  Eigen::Affine3d GetCurrentNodePose(bool use_obs_centroid=false);

  Eigen::Affine3d GetPreviousNodePose(bool use_obs_centroid=false);

  virtual std::string ToString();

  mapNodeItr begin(){return map_nodes_.begin();} /* e.g. for(mapNodeItr itr=graph_map->begin(); itr!=graph_map->end();itr++) */

  mapNodeItr end(){return map_nodes_.end();}

  //!
  //! \brief GetCloud Get the pointcloud which was used to update all map nodes
  //! \param cloud the point cloud
  //! \param offset transformation of all points
  //!
  void GetCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, bool original_points);


//<<<<<<< HEAD
//  //virtual std::vector<FactorPtr> GetFactors(NodePtr node);//Get all factors for current node
//  GraphMap();
//
//  //Accessors
//  bool UseSubmaps() const {return use_submap_;}
//  double InterchangeRadius() const {return interchange_radius_;}
//  double CompoundRadius() const {return compound_radius_;}
//  double MinKeyframeDist() const {return min_keyframe_dist_;}
//  double MinKeyframeRotDeg() const {return min_keyframe_rot_deg_;}
//  bool UseKeyframe() const {return use_keyframe_;}

//	NodePtrVector GetNodes(){return nodes_;}


  NodePtrVector GetNodes(){return nodes_;}
  const NodePtrVector& GetNodes() const {return nodes_;}
  std::vector<MapNodePtr> GetMapNodes(){return map_nodes_;}
  const std::vector<MapNodePtr>& GetMapNodes() const {return map_nodes_;}
  std::vector<FactorPtr> GetFactors(){return factors_;}
  const std::vector<FactorPtr>& GetFactors() const {return factors_;}
  

protected:

  MapNodePtr currentNode_=NULL,prevNode_=NULL;//The current node
	NodePtrVector nodes_;//Vector of all nodes in graph
  std::vector<FactorPtr> factors_;
  std::vector<MapNodePtr> map_nodes_;
  MapParamPtr mapparam_=NULL;

private:
  friend class GraphFactory;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & currentNode_;
    ar & prevNode_;
    ar & nodes_;
    ar & map_nodes_;
    ar & factors_;
  }

};

void SaveGraphMapPLY(const std::string &filename, GraphMapPtr graph_map, bool original_points=true);

class GraphMapParam{
public:
  void GetParametersFromRos();
  unsigned int max_size=0;
  GraphMapParam();
  virtual ~GraphMapParam() { }
private:
  friend class GraphFactory;
};
}
}
#endif // GRAPH_H
