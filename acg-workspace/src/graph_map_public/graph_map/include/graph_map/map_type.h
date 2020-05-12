/**
 *  file map_type.h.
 */
#ifndef MAPTYPE_H
#define MAPTYPE_H
#include "graphfactory.h"
#include "eigen3/Eigen/Dense"
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "boost/serialization/serialization.hpp"
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <velodyne_pointcloud/point_types.h>
#include "ndt_generic/eigen_utils.h"
#include "ndt_generic/serialization.h"



namespace perception_oru{
namespace graph_map{

//    using namespace std;
/*!
 * ... Abstract class to present parameters for "mapType". this class contain generic parameters for all map types.  ...
 */





/*!
 * ... Abstract class to implement local maps.  ...
 */

class MapType{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapType();
  /*!
   * \brief update attempts to update the map based on point cloud data and the pose where the scan was taken from(scanner) in the world fram
   * \param Tnow transformation from world to sensor frame
   * \param cloud data to update map with
   */
  bool Initialized() const{return initialized_;}
  /*!
   * \brief updateMap updates the map with could given Tnow iff enable_mapping_
   * \param Tnow the source of cloud
   * \param cloud is the point cloud used to map update the map
   * \param simple - Henrik please comment on this parameter.
   */
  virtual void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple = false);

  /*!
   * \brief updateMap updates the map with could given Tnow iff enable_mapping_
   * \param Tnow the source of cloud
   * \param cloud is the point cloud used to map update the map - includes additional information about intensity of measurments
   * \param simple - Henrik please describe this parameter.
   */
  virtual void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple = false);

  virtual bool CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius=5);

  virtual std::string GetMapName()const{return mapName_;}

  virtual std::string ToString();

  Eigen::Affine3d& GetObservationCentroid(){return mean_obs_location_;} //in local_map_frame

  ndt_generic::Affine3dSTLVek  GetObservationVector(){return pose_est_;} //in local_map_frame

  uint64_t GetStamp() const;

  virtual void GetCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, bool original_points=true);


protected:

  virtual void update(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple = false)=0;

  virtual void update(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple = false)=0;


  MapType(MapParamPtr param);
  double sizex_=0;
  double sizey_=0;
  double sizez_=0;
  double max_range_=130;
  double min_range_=0.6;
  bool initialized_=false;
  bool enable_mapping_=true;
  bool store_points_=false;
  uint64_t stamp;
  std::string mapName_="";
  ndt_generic::Affine3dSTLVek pose_est_;
  Eigen::Affine3d mean_obs_location_=Eigen::Affine3d::Identity();
  unsigned int observations_=0;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > input_cloud_;

  //pcl::octree::OctreePointCloudSinglePoint<pcl::PointXYZ> octree;mapParPtr

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & sizex_ & sizey_ & sizez_;
    ar & max_range_ & min_range_;
    ar & initialized_;
    ar & enable_mapping_;
    ar & mapName_;
    ar & mean_obs_location_;
    ar & pose_est_;
    ar & input_cloud_;
  }
};

class MapParam{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~MapParam()=0;
	std::string getMapName() const{return mapName_;}
  virtual void GetParametersFromRos();
  virtual std::string ToString();
  double sizex_=150;
  double sizey_=150;
  double sizez_=12;
  double max_range_=130;
  double min_range_=0.6;
  bool enable_mapping_=true;
  bool store_points=false;
protected:
  MapParam(){}
	std::string mapName_="";
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & sizex_;
    ar & sizey_;
    ar & sizez_;
    ar & max_range_;
    ar & min_range_;
    ar & enable_mapping_;
  }
};
}
}
#endif // MAPTYPE_H
