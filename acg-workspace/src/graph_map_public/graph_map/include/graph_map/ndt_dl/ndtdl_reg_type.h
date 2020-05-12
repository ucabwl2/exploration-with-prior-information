#ifndef TEMPLATE_REG_TYPE_H
#define TEMPLATE_REG_TYPE_H

#include "graph_map/graphfactory.h"
#include "graph_map/reg_type.h"
#include "graph_map/map_type.h"
#include "graph_map/ndt_dl/ndtdl_map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include "math.h"
#include "ros/ros.h"
#include "ros/node_handle.h"


#define ndt_dl_reg_type_name "ndt_dl_reg"
namespace perception_oru{
namespace graph_map{

class NDTDLRegTypeParam:public registrationParameters{
public:
  ~NDTDLRegTypeParam();
  void GetParametersFromRos();//Get parametes from ros e.g. from the ros parameter server
  //but all your parameters here
 std::string super_important_parameter_;
protected:
  NDTDLRegTypeParam();//Constructor is protected to allow only graphcatory to instanciate or derived classes create this type
private:
  friend class GraphFactory;

};

class NDTDLRegType:public registrationType{
public:
  ~NDTDLRegType();
  template<class PointT> bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess
protected:
  std::string super_important_parameter_;
  NDTDLRegType(RegParamPtr paramptr);
private:
  friend class GraphFactory;
};



}
}



#endif // TEMPLATE_REG_TYPE_H
