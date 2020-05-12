#ifndef NARF_REG_TYPE_H
#define NARF_REG_TYPE_H

#include "graph_map/graphfactory.h"
#include "graph_map/reg_type.h"
#include "graph_map/map_type.h"
#include "graph_map/template/template_map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include "math.h"
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "graph_map/narf/narf_map_type.h"


#define narf_reg_type_name "narf_reg"
namespace perception_oru{
namespace graph_map{
class NarfRegType:public registrationType{
public:
  ~NarfRegType();
  bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Eigen::MatrixXd &Tcov);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess
protected:
    std::string super_important_parameter_;
  NarfRegType(RegParamPtr paramptr);
private:
  friend class GraphFactory;
};


class NarfRegTypeParam:public registrationParameters{
public:
  ~NarfRegTypeParam();
  void GetParametersFromRos();//Get parametes from ros e.g. from the ros parameter server
  //but all your parameters here
  std::string super_important_parameter_;
protected:
  NarfRegTypeParam();//Constructor is protected to allow only graphcatory to instanciate or derived classes create this type
private:
  friend class GraphFactory;

};
}

}



#endif // TEMPLATE_REG_TYPE_H
