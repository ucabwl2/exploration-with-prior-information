#ifndef READPOINTCLOUD_IMPL_H
#define READPOINTCLOUD_IMPL_H
using std::string;
namespace  ndt_offline {

template<class PointT>
bool readPointCloud::UnwarpVelodyneMsgImuUndistortion(velodyne_msgs::VelodyneScan::ConstPtr &scan,pcl::PointCloud<PointT> &cloud){
  Eigen::Affine3d Tdiff=Eigen::Affine3d::Identity();
  Eigen::Matrix3d Tnow,T_first_seg;
  ros::Time t_first;
  velodyne_rawdata::VPointCloud pnts;
  pcl::PointCloud<PointT> cloud_segments,tmp;
  t_first=scan->packets[0].stamp;
  T_first_seg=(imu_pred_->GetOrientation(scan->packets[0].stamp.toSec()+t_offset_).toRotationMatrix()*Tsensor_.linear());//Get the transformation of the imu at the start of the pointclod

  for(size_t i=0;i<scan->packets.size();i++){
    Tnow=imu_pred_->GetOrientation(scan->packets[i].stamp.toSec()+t_offset_).toRotationMatrix()*Tsensor_.linear();
    Tdiff.linear()=Tnow.inverse()*T_first_seg;
    velodyne_parser_->unpack(scan->packets[i], pnts); // unpack the raw data
    pcl::copyPointCloud(pnts,cloud_segments);

    pcl::transformPointCloud(cloud_segments,tmp,Tdiff);
    cloud+=tmp;
    pnts.clear();
    cloud_segments.clear();
  }
  cloud.header.frame_id=scan->header.frame_id;
  pcl_conversions::toPCL(scan->header.stamp,cloud.header.stamp);
  cloud.width=cloud.size();
  cloud.height=1;
  return cloud.size()>0;
}

template<class PointT>
bool readPointCloud::UnwarpVelodyneMsgOdomUndistortion(velodyne_msgs::VelodyneScan::ConstPtr &scan,pcl::PointCloud<PointT> &cloud){

  cloud.clear();
  Eigen::Affine3d T_odom_t0,T_odom_tnow,Tsensor_t0,Tsensor_tnow;
  tf::Transform T_tf;
  ros::Time t0=scan->header.stamp + ros::Duration(t_offset_);

  if(odom_lookup->getTransformationForTime(t0, interpolation_link_id_, T_odom_t0)){
    Tsensor_t0=T_odom_t0*Tsensor_;

    for (size_t next = 0; next < scan->packets.size(); ++next){
      velodyne_rawdata::VPointCloud pnts,conv_points;
      velodyne_parser_->unpack(scan->packets[next], pnts); // unpack the raw data
      ros::Time t1=scan->packets[next].stamp + ros::Duration(t_offset_);
      if(odom_lookup->getTransformationForTime(t1,interpolation_link_id_,T_odom_tnow)){
        Tsensor_tnow=T_odom_tnow*Tsensor_;
        Eigen::Affine3d T=(Tsensor_t0.inverse()*Tsensor_tnow);
        tf::poseEigenToTF(T,T_tf);
        pcl_ros::transformPointCloud(pnts,conv_points,T_tf);
        convertPointCloud(conv_points, cloud);
      }
      else{
        cloud.clear();
        return false;
      }
      pnts.clear();
      conv_points.clear();
    }
  }
  else{
    std::cout<<"Unwarp without interpolation"<<std::endl;
    ndt_generic::UnwarpCloudSimple(*velodyne_parser_,scan,cloud);
  }

  cloud.header.frame_id=scan->header.frame_id;
  pcl_conversions::toPCL(scan->header.stamp,cloud.header.stamp);
  cloud.width=cloud.size();
  cloud.height=1;
  return cloud.size()>0;
}

template<class PointT>
bool readPointCloud::readNextMeasurement(pcl::PointCloud<PointT> &cloud){

  bool cloud_found=false;
  pcl::PointCloud<PointT> cloud_unfiltered;
  cloud.clear();
  while(I!=view_->end() && !cloud_found){
    cloud.clear();
    cloud_unfiltered.clear();
    rosbag::MessageInstance const m = *I;
    if(m.getTopic()==lidar_topic_){
      if (velodyne_msgs::VelodyneScan::ConstPtr scan = m.instantiate<velodyne_msgs::VelodyneScan>()){
        if( odometry_sensor_== WHEEL_ODOM){
          //std::cout<<"WHEEL_ODOM"<<std::endl;
          cloud_found=UnwarpVelodyneMsgOdomUndistortion<PointT>(scan,cloud);
          //cloud_found=ndt_generic::UnwrapCloudSimple<PointT>(*velodyne_parser_,scan,cloud);
        }
        else if( odometry_sensor_== IMU){
          std::cout<<"IMU"<<std::endl;
          cloud_found=UnwarpVelodyneMsgImuUndistortion<PointT>(scan,cloud);
        }
        else if(odometry_sensor_== NO_ODOM){
          //std::cout<<"NO_ODOM"<<std::endl;
          cloud_found=ndt_generic::UnwarpCloudSimple<PointT>(*velodyne_parser_,scan,cloud);
        }
        pcl_conversions::fromPCL(cloud.header.stamp,t_cloud);
      }
      else{
        if( sensor_msgs::PointCloud2ConstPtr point_cloud2 = m.instantiate<sensor_msgs::PointCloud2>() ){
          cloud_found=ndt_generic::MsgCloud2ToPCL<PointT>(point_cloud2,cloud_unfiltered);
          if(odometry_sensor_== WHEEL_ODOM)
            UnwarpCloudWheelOdom(cloud_unfiltered);
        }
        else if(sensor_msgs::PointCloudConstPtr point_cloud = m.instantiate<sensor_msgs::PointCloud>()){
          cloud_found=ndt_generic::MsgCloudToPCL<PointT>(point_cloud,cloud_unfiltered);
          if(odometry_sensor_== WHEEL_ODOM)
            UnwarpCloudWheelOdom(cloud_unfiltered);
        }
        else if( sensor_msgs::LaserScanConstPtr laser_scan = m.instantiate<sensor_msgs::LaserScan>() ){
          cloud_found=ndt_generic::LaserScanToPCL<PointT>(laser_scan,cloud_unfiltered);
          ndt_generic::AddVariance(cloud_unfiltered);
        }
        if(cloud_found)
          ndt_generic::filter_range_fun(cloud,cloud_unfiltered,min_range_,max_range_);
        t_cloud=m.getTime();
      }


    }

    I++;
  }
  return I!=view_->end();
}

template<class PointT> void readPointCloud::convertPointCloud(const velodyne_rawdata::VPointCloud &conv_points,
                                                              pcl::PointCloud<PointT> &cloud) {
  for(size_t i = 0;i<conv_points.size();i++){
    PointT p;
    p.x =conv_points.points[i].x; p.y=conv_points.points[i].y; p.z=conv_points.points[i].z;
    cloud.push_back(p);
  }
  cloud.header.frame_id=conv_points.header.frame_id;
  cloud.header.stamp=conv_points.header.stamp;
  cloud.width=cloud.size();
  cloud.height=1;
}
template<class PointT> void readPointCloud::UnwarpCloudWheelOdom(pcl::PointCloud<PointT> &cloud){
  ros::Time t0;
  Eigen::Affine3f Tsensor_t0;
  Eigen::Affine3d T_odom_t0;
  Eigen::Affine3d T_odom_t;
  Eigen::Affine3f Tsensor_t;

  const double scan_time=0.1;
  pcl_conversions::fromPCL(cloud.header.stamp,t0);
  pcl::PointCloud<PointT> unwarped;
  if(cloud.size()==0 || !odom_lookup->getTransformationForTime(t0+ros::Duration(t_offset_), interpolation_link_id_, T_odom_t0)||
                        !odom_lookup->getTransformationForTime(t0+ros::Duration(scan_time)+ros::Duration(t_offset_), interpolation_link_id_, T_odom_t))
    return;

  Tsensor_t=(T_odom_t*Tsensor_).cast<float>();
  Tsensor_t0=(T_odom_t0*Tsensor_).cast<float>();
  //Tsensor_t0=Eigen::Affine3f::Identity();
  std::cout<<"toffset="<<t_offset_<<std::endl;
  PointT p0=cloud[0];
 #pragma omp parallel num_threads(6)
  {
    #pragma omp for
    for(unsigned int i=0; i<cloud.size()-1; i++){
      double angle=-atan2(p0.x*cloud[i].y-p0.y*cloud[i].x, p0.x*cloud[i].x+p0.y*cloud[i].y);
      if(angle<0)
        angle=angle+2*M_PI;


      Eigen::Affine3f T=Tsensor_t0.inverse()*ndt_generic::Interpolate(Tsensor_t0,Tsensor_t,angle/(2*M_PI)).cast<float>();
      //Tsensor_t=Eigen::Affine3f::Identity();
      Eigen::Map<Eigen::Vector3f> pt((float*)&cloud.points[i],3);
      pt = T*pt;
    }
  }

}

template<> void readPointCloud::convertPointCloud(const velodyne_rawdata::VPointCloud &conv_points,
                                                  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud) {
  for(size_t i = 0;i<conv_points.size();i++){
    velodyne_pointcloud::PointXYZIR p;
    p.x =conv_points.points[i].x; p.y=conv_points.points[i].y; p.z=conv_points.points[i].z;
    p.intensity = conv_points.points[i].intensity;
    p.ring = conv_points.points[i].ring;
    cloud.push_back(p);
  }
  cloud.header.frame_id=conv_points.header.frame_id;
  cloud.header.stamp=conv_points.header.stamp;

  cloud.width=cloud.size();
  cloud.height=1;
}


}
#endif // READPOINTCLOUD_IMPL_H
