#ifndef MAP_NODE_H
#define MAP_NODE_H
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "stdio.h"
#include <iostream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "boost/serialization/shared_ptr.hpp"
#include "ndt_generic/serialization.h"
#include <velodyne_pointcloud/point_types.h>
#include "ndt_generic/eigen_utils.h"
namespace perception_oru{
namespace graph_map{
/*!
* ... Class to represent a node ...
*/

class Node{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Node();

    virtual const Affine3d& GetPose() const;
    virtual Affine3d& GetPose();

  virtual Affine3d SetPose(const Eigen::Affine3d &pose){pose_=pose;}



  bool operator ==(const Node& node_compare);

  virtual unsigned int GetId()const{return id_;}

  virtual std::string ToString(){return "base node";}

  virtual bool WithinRadius(const Affine3d &pose, const double &radius);

  static bool DetectNewNode(unsigned int &node_id, NodePtr node); //compare node with node_id, if different, node.GetId is assigned to node_id

protected:

  unsigned int id_;
  Eigen::Affine3d pose_;

private:

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & id_;
    ar & pose_;
  }
};
/*!
* ... Class to represent a map node ...
*/
class MapNode:public Node{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapNode();

  void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud);

  void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud);

  virtual std::string ToString();

  virtual MapTypePtr GetMap(){return map_;}

  virtual bool Initialized(){return initialized_;}

  Eigen::Affine3d GetMapPose(bool obs_centroid=false);

  ndt_generic::Affine3dSTLVek GetObservationVector(bool local_frame=false);

  bool WithinRadius(const Affine3d &pose, const double &radius, bool obs_centroid=false);

protected:
  MapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam);
  MapTypePtr map_=NULL;
  bool initialized_=false;
private:
  friend class GraphFactory;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & boost::serialization::base_object<Node>(*this);
    ar & map_;
    ar & initialized_;
  }


};
}

}
#endif // MAP_NODE_H
