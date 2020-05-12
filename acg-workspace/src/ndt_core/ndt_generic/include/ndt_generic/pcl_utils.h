#pragma once

#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>
#include <velodyne_pointcloud/rawdata.h>
#include "velodyne_msgs/VelodyneScan.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include "laser_geometry/laser_geometry.h"
#include "queue"
#include "ndt_generic/utils.h"
#include "pcl/io/pcd_io.h"
namespace ndt_generic {

template <class PointT>
bool MsgCloud2ToPCL( sensor_msgs::PointCloud2ConstPtr &point_cloud2, pcl::PointCloud<PointT> &cloud);

template <class PointT>
bool MsgCloudToPCL( sensor_msgs::PointCloudConstPtr &point_cloud, pcl::PointCloud<PointT> &cloud);

template <class PointT>
bool LaserScanToPCL( sensor_msgs::LaserScanConstPtr &laser_scan, pcl::PointCloud<PointT> &cloud);

template <class PointT>
bool UnwarpCloudSimple( velodyne_rawdata::RawData &velodyneparser, const velodyne_msgs::VelodyneScan::ConstPtr& msg_in, pcl::PointCloud<PointT> &cloud);

pcl::PointXYZ eigenToPCLPoint(const Eigen::Vector3d &pt);

Eigen::Vector3d PCLPointToEigen(const pcl::PointXYZ &pt);

void computeDirectionsAndRangesFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector3d &origin,
    std::vector<Eigen::Vector3d> &dirs, std::vector<double> &ranges);

template <class PointT>
void filter_fov_fun(pcl::PointCloud<PointT> &cloud,
                    pcl::PointCloud<PointT> &cloud_nofilter, double hori_min,
                    double hori_max);

template <class PointT>
void filter_range_fun(pcl::PointCloud<PointT> &cloud,
                      pcl::PointCloud<PointT> &cloud_nofilter, double range_min,
                      double range_max);

template <class PointT>
void filter_height_angle(pcl::PointCloud<PointT> &cloud,
                         double hori_min=-2*M_PI,
                         double hori_max=2*M_PI, double min_z=DBL_MIN,
                         double max_z=DBL_MAX);

template <typename PointT>
void getMinMax3DPointCloud(const pcl::PointCloud<PointT> &cloud,
                           Eigen::Vector3d &minP, Eigen::Vector3d &maxP);

template <class PointT>
void filter_height_angle(pcl::PointCloud<PointT> &cloud,
                         pcl::PointCloud<PointT> &cloud_nofilter,
                         double hori_min=-2*M_PI,
                         double hori_max=2*M_PI,
                         double min_z=DBL_MIN,
                         double max_z=DBL_MAX);

template <class PointT>
void AddVariance(pcl::PointCloud<PointT> &cloud, double varz=0.05);


template <class PointT>
class PointCloudQueue
{
public:

  PointCloudQueue(size_t size=10) {max_size_=size;}

  void Push(const pcl::PointCloud<PointT> &cloud);

  void GetCloud(pcl::PointCloud<PointT> &cloud);

  void Save( const std::string &name_prefix="cloud_");

private:
  size_t max_size_;
  std::vector<pcl::PointCloud<PointT> > clouds;
  unsigned int cloud_set_=0;

};

  } // namespace

#include "ndt_generic/pcl_utils_impl.h"

