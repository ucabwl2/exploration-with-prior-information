#include "graph_map/narf/narf_map_type.h"
#include <boost/serialization/export.hpp>
//BOOST_CLASS_EXPORT(perception_oru::libgraphMap::NarfMapType)
namespace perception_oru{
namespace graph_map{

using namespace std;


NarfMapType::NarfMapType( MapParamPtr paramptr) : MapType(paramptr), viewer("3D Viewer"), coordinate_frame(pcl::RangeImage::CAMERA_FRAME),range_image_widget ("Range image"),t_viewer(&NarfMapType::NarfViewer,this){
 viewer.setBackgroundColor (1, 1, 1);
  NarfMapParamPtr param = boost::dynamic_pointer_cast< NarfMapParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Get parameters to this class from param
    cout<<"templateMapType: created templateMapType"<<endl;
      viewer.setBackgroundColor (1, 1, 1);
  }
  else
    cerr<<"templateMapType: Cannot create instance for \"templateMapType\""<<std::endl;
}
NarfMapType::~NarfMapType(){}
void NarfMapType::update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple){//Mandatory, base method implemented as pure virtual}
  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(cloud, cloud_xyz);
  update(Tsensor, cloud_xyz);
}
void NarfMapType::NarfViewer(){
  while(!viewer_updated){
    pcl_sleep(0.1);
  }
  while (!viewer.wasStopped ())
  { m.lock();
    range_image_widget.spinOnce ();  // process GUI events
    viewer.spinOnce ();
    m.unlock();
    pcl_sleep(0.01);
  }
}

void NarfMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud,bool simple){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.
  static bool first_time=true;
  if(!initialized_){
    initialized_=true;
    return;
  }
ros::Time T1=ros::Time::now();

  cout<<"cloud size:"<<cloud.size()<<endl;
  float support_size = 1.0f;
  bool setUnseenToMaxRange = false;
  bool rotation_invariant = true;
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud (cloud, pcl::deg2rad (0.16f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   Eigen::Affine3f(Tsensor), coordinate_frame, noise_level, min_range, border_size);
  //range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);

  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  if(first_time){
 // viewer.initCameraParameters ();
  //setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
  }

  // --------------------------
  // -----Show range image-----
  // --------------------------




  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;

  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

  // ----------------------------------------------
  // -----Show keypoints in range image widget-----
  // ----------------------------------------------
  //for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    //range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
                                  //keypoint_indices.points[i]/range_image.width);

  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.points.resize (keypoint_indices.points.size ());
  for (size_t i=0; i<keypoint_indices.points.size (); ++i)
    keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);


  // ------------------------------------------------------
  // -----Extract NARF descriptors for interest points-----
  // ------------------------------------------------------
  std::vector<int> keypoint_indices2;
  keypoint_indices2.resize (keypoint_indices.points.size ());
  for (unsigned int i=0; i<keypoint_indices.size (); ++i) // This step is necessary to get the right vector type
    keypoint_indices2[i]=keypoint_indices.points[i];
  pcl::NarfDescriptor narf_descriptor (&range_image, &keypoint_indices2);
  narf_descriptor.getParameters ().support_size = support_size;
  narf_descriptor.getParameters ().rotation_invariant = rotation_invariant;
  pcl::PointCloud<pcl::Narf36> narf_descriptors;
  narf_descriptor.compute (narf_descriptors);
  cout << "Extracted "<<narf_descriptors.size ()<<" descriptors for "
                      <<keypoint_indices.points.size ()<< " keypoints.\n";
  first_time=false;
  cout<<"narf time="<<ros::Time::now()-T1<<endl;
  m.lock();

  viewer.removePointCloud("range image");
  viewer.removePointCloud("keypoints");
  viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
  range_image_widget.showRangeImage (range_image);
   m.unlock();
  viewer_updated=true;

  //narf_descriptor.

  //--------------------
  // -----Main loop-----
  //--------------------

}
void NarfMapType::setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}
bool NarfMapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){
  cout<<"please implement map compound for improved usage of submaps"<<endl;
  return true;//remove this
  if( NarfMapTypePtr targetPtr=boost::dynamic_pointer_cast<NarfMapType>(target) ){

    cout<<"\"CompoundMapsByRadius\" not overrided by template but not implemented"<<endl;
  }
}


void NarfMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
  nh.param<std::string>("Super_important_map_parameter",SuperImportantMapParameter,"parhaps not so important...");
}
}


}
