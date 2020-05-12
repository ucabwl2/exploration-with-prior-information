#ifndef NDT2DMAP_TYPE_H
#define NDT2DMAP_TYPE_H
#include "graph_map/graphfactory.h"
#include <graph_map/map_type.h>
#include <graph_map/ndt/ndt_map_param.h>
#include <ndt_map/ndt_map_hmt.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include "graph_map/graph_plot.h"
#include <ndt_map/pointcloud_utils.h>
//#include <ndt_fuser/motion_model_2d.h>
#include "stdio.h"
#include "sstream"
#include "eigen_conversions/eigen_msg.h"
#define ndt_map_type_name "ndt_map"
namespace perception_oru{
namespace graph_map{
using namespace perception_oru;

class NDTMapType:public MapType{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~NDTMapType();
  virtual void update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple = false);

  virtual void update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple = false);

  virtual NDTMap* GetNDTMap() { return map_;}

  virtual void GetCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, bool original_points=true);

  //Advanced
  virtual bool CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius);
    std::string ToString();
  double GetResolution() const{return resolution_;}
  NDTMapType(MapParamPtr paramptr);
  NDTMapType(){}
  NDTMap *map_=NULL;

protected:
  double resolution_=0.4;
  double resolution_local_factor_=1.;
  bool ndt_cell_cloud_=false;
  bool ndt_filtered_cloud_=true;

  friend class GraphFactory;
  void InitializeMap(const Eigen::Affine3d &Td,pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple = false);

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & boost::serialization::base_object<MapType>(*this);
    ar & map_;
    ar & resolution_;
    ar & resolution_local_factor_;
  }
  /*-----End of Boost serialization------*/

};


}
}
#endif // NDTMAP_TYPE_H
