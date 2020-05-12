#ifndef GRAPH_NAVIGATOR_INTERFACE_H
#define GRAPH_NAVIGATOR_INTERFACE_H
#include "graph_map/graph_map.h"
#include "graph_map/map_node.h"
#include "graph_map/map_type.h"
#include "graphfactory.h"
#include "Eigen/Dense"
#include <boost/serialization/base_object.hpp>
#include "boost/serialization/serialization.hpp"
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include "boost/serialization/shared_ptr.hpp"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "ndt_generic/eigen_utils.h"

namespace perception_oru{
namespace graph_map{
using namespace perception_oru;

typedef enum SelectMap{node_position=0,mean_observation=1,closest_observation=2, grid=3,node_position_esg=4, overlap=5}MapSwitchingMethod;

class GraphMapNavigator:public GraphMap{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraphMapNavigator(const Eigen::Affine3d &nodepose, const MapParamPtr &mapparam, const GraphMapParamPtr graphparam);

  GraphMapNavigator();

  std::string ToString();

  //!
  //! \brief GetMapByOverlap computes the closest map depeing on hte overlap between current scan and a map in vicinity of the sensor
  //! \param Tnow the pose of the sensor
  //! \return pointer to closest map
  //!
  MapNodePtr GetMapByOverlap(const Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> *cloud=NULL);

  MapNodePtr GetMapNodeByObservationHistory(const Eigen::Affine3d &Tnow);

  MapNodePtr GetMapNodeByObservationHistory(const Eigen::Affine3d &Tnow, unsigned int n_search);

  std::vector<MapNodePtr> GetClosestNodes(const Eigen::Affine3d &Tnow, double max_distance, bool factor_interchange=false); //if factor_interchange==true, max distance equals interchange_radius*max_distance

  void UpdateGraph(const Eigen::Affine3d &pose, pcl::PointCloud<pcl::PointXYZ> &cloud);

  void WorldToLocalMapFrame(Eigen::Affine3d &pose, MapNodePtr frame=NULL);
  void LocalToWorldMapFrame(Eigen::Affine3d &pose, MapNodePtr frame=NULL);

  bool SwitchToClosestMapNode(const Affine3d &Tnow, double max_distance=DBL_MAX, pcl::PointCloud<pcl::PointXYZ> *cloud=NULL);
  //!
  //! \brief SwitchToClosestMapNode Attempts to find the closest Mapnode within desired radius
  //! \param Tnow is the position around the closest map will be found
  //! \param cov not used5
  //! \param T_world_to_local_map not used
  //! \param radius the maximum distance to search within, 0.0 allows nodes at any distance
  //! \return true if a node was found within range

  //bool SwitchToClosestMapNode(const Eigen::Affine3d &Tnow, const Matrix6d &cov, const double radius);
  //!
  //! \brief SwitchToClosestMapNode returns the closest map node, return true if a node was fund
  //! \param Tnow is the target pose in the global frame
  //! \param use_map_centroid
  //! \return true if a node was found
  //!

  void SetMapSwitchMethod(MapSwitchingMethod map_switch_method=node_position){map_switch_method_=map_switch_method;}

  bool AutomaticMapInterchange(const Eigen::Affine3d &Tnow, const Matrix6d &cov, bool &changed_map_node, bool &created_map_node );

  void AddMapNode(const Eigen::Affine3d &diff, const Matrix6d &cov=unit_covar){GraphMap::AddMapNode(diff,cov); map_nodes_grid_.SetVal(currentNode_,GetCurrentNodePose().translation());}

  bool SwitchToMapNode(MapNodePtr new_node);

  const Eigen::Affine3d& GetSensorPose() const {return Tsensor_;}
  Eigen::Affine3d& GetSensorPose() {return Tsensor_;}

protected:
  bool use_submap_=false;
  double interchange_radius_=0;
  double compound_radius_=0;
  bool use_keyframe_=true;
  double min_keyframe_dist_=0.5;
  double min_keyframe_rot_deg_=15;

  VoxelGrid<MapNodePtr> map_nodes_grid_;
  MapSwitchingMethod map_switch_method_=node_position;
 //distance metrics
  double alpha_=0.0;
  double k_=0.0;
  unsigned int n_search_=5;
  Eigen::Affine3d Tsensor_;

private:



  MapNodePtr GetClosestMapNode(const Eigen::Affine3d &Tnow, const bool use_observation_centroid=false);

  bool TransitionGrid(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node);

  bool TransitionSG(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node);

  bool TransitionESG(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node);

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)//In order to clal this you need to register it to boost using "ar.template register_type<LazyGrid>();"
  {
    ar & boost::serialization::base_object<GraphMap>(*this);
    ar & use_submap_;
    ar & interchange_radius_;
    ar & compound_radius_;
    ar & use_keyframe_;
    ar & min_keyframe_dist_;
    ar & min_keyframe_rot_deg_;
    ar & map_nodes_grid_;
    ar & alpha_;
    ar & k_;
    ar & Tsensor_;
  }
};

bool LoadGraphMap(const std::string &file_name,  GraphMapNavigatorPtr &ptr);
void SaveObservationVector(const std::string &file_name, GraphMapNavigatorPtr graph_map);


class GraphMapNavigatorParam:public GraphMapParam{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraphMapNavigatorParam(){}
  /*!
   * \brief ParseSetMapSwitchMethod
   * \param switch_method "mean_observation" "closest_observation" or default("node_position")
   * \return
   */
  static MapSwitchingMethod String2SwitchMethod(const std::string &switch_method);

  static std::string SwitchMethod2String(const MapSwitchingMethod &switch_method);

  void GetParametersFromRos();
  bool use_submap=false;
  bool use_keyframe=true;
  double interchange_radius=0;
  double compound_radius=0;
  double min_keyframe_dist=0.5;
  double min_keyframe_rot_deg=15;
  double alpha=0.0;//use angular distance
  unsigned int n_search=5;
  Eigen::Affine3d Tsensor = Eigen::Affine3d::Identity();

  MapSwitchingMethod map_switch_method=node_position;

private:
  friend class GraphFactory;

};

}
}
#endif // GRAPH_NAVIGATOR_INTERFACE_H
