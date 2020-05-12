
#include "graph_map/graph_map_fuser.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ndt_map/ndt_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>


#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#include <ndt_map/NDTMapMsg.h>
#include "ndt_generic/eigen_utils.h"
//#include <ndt_generic/gnuplot-iostream.h>
#include "graph_map/lidarUtils/lidar_utilities.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "boost/serialization/serialization.hpp"
#include <boost/program_options.hpp>
#include "graph_map/graph_plot.h"
#include "ros/publisher.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"


namespace po = boost::program_options;
using namespace perception_oru;
using namespace graph_map;
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */


class OpenGraphMap {

protected:
  // Our NodeHandle

public:
  // Constructor
  OpenGraphMap(ros::NodeHandle *param_nh,const string &file_name)
  {
    param_nh_=param_nh;
    LoadGraphMap(file_name,graph_map_);
    robot_pose_pub_= param_nh_->advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
    sub=param_nh_->subscribe("/initialpose",1,&OpenGraphMap::pose_callback,this);
    marker_ = plotmarker::point;
  }


  void processFrame(){
    bool action=false;
    m.lock();
    if(got_pose_target){
      graph_map_->SwitchToClosestMapNode(pose_target);
      got_pose_target=false;
      visualize();
      action=true;
    }
    else if(got_keyboard_target){
      graph_map_->SwitchToMapNode(graph_map_->GetNode(target_node));
      got_keyboard_target=false;
      visualize();
      action=true;
    }
    /*if(Save_submap_metadata_){
      saveSubmapMetadata();
      Save_submap_metadata_=false;
      action=true;
    }*/
    m.unlock();

  }

  void SetFilePath(std::string &path){output_path=path;}

  /*void saveSubmapMetadata(){
  std::ofstream stream_sensor,stream_submaps;
  std::string path_sensor,path_submaps;
  path_sensor=output_path+"_sensorpath.txt";
  path_submaps=output_path+"_submaps.txt";
  stream_sensor.open(path_sensor.c_str());
  stream_submaps.open(path_submaps.c_str());
  std::cout<<"output files:"<<path_sensor<<std::endl;
  std::cout<<"output files:"<<path_submaps<<std::endl;
  if( GraphMapNavigatorPtr graph=boost::dynamic_pointer_cast<GraphMapNavigator>(graph_map_)){
     for(int i=0;i<graph->Size();i++){
      Eigen::Affine3d node = graph->GetNodePose(i);
      Eigen::Vector3d rot = node.rotation().eulerAngles(0,1,2);
      Eigen::Vector3d position=node.translation();
      ndt_generic::normalizeEulerAngles(rot);
      stream_submaps << i<<" "<<position(0)<<" "<<position(1)<<""<<position(2)<<" "<<rot(2)<<std::endl;
      stream_submaps.flush();
     }
    for(int i=0;i<graph->Size();i++){
        ndt_generic::Affine3dSTLVek obs_vec=graph->GetNode(i)->GetObservationVector();
        for(int j=0;j<obs_vec.size();j++){
          Eigen::Vector3d rot = obs_vec[j].rotation().eulerAngles(0,1,2);
          Eigen::Vector3d position=obs_vec[j].translation();
          ndt_generic::normalizeEulerAngles(rot);
          stream_sensor << i<<" "<<position(0)<<" "<<position(1)<<""<<position(2)<<" "<<rot(2)<<std::endl;
          stream_sensor.flush();
        }
    }
  }
  stream_sensor.close();
  stream_submaps.close();
  }*/

  void visualize(){
    GraphPlot::PlotPoseGraph(graph_map_);
    GraphPlot::PlotObservationVector(graph_map_);
    GraphPlot::PlotMap(graph_map_->GetCurrentNode()->GetMap(), -1, graph_map_->GetCurrentNodePose(), marker_);
  }
  void setMarker(PlotMarker marker) {
    marker_ = marker;
  }
  void KeyboardInputThread(){

  char input=' ';
    do
  {
    cin>>input;
    if(input=='d'){
      target_node++;
    }
    else if(input=='a'){
      target_node--;
    }
    else if(input=='p'){
      std::cout<<"Requested saving metadata"<<std::endl;
      Save_submap_metadata_=true;
    }
    else if(input=='h'){
      //std::cout<<help_text<<std::endl;
    }

  if(  target_node==graph_map_->Size())
    target_node=0;
  else if(target_node==-1)
    target_node=graph_map_->Size()-1;
    std::cout<<"target_node="<<target_node<<"/"<<graph_map_->Size()<<std::endl;
    got_keyboard_target=true;
  }while(input!='e');

  }

  void pose_callback (const geometry_msgs::PoseWithCovarianceStamped& target_pose){
    geometry_msgs::PoseStamped msg_pose;
    msg_pose.pose.orientation=target_pose.pose.pose.orientation;
    msg_pose.pose.position=target_pose.pose.pose.position;
    msg_pose.header.stamp=ros::Time::now();
    msg_pose.header.frame_id="/world";
    robot_pose_pub_.publish(msg_pose);
    m.lock();
    tf::poseMsgToEigen(target_pose.pose.pose,pose_target);
    got_pose_target=true;
    m.unlock();
  }

private:
  GraphMapNavigatorPtr graph_map_;
  ros::NodeHandle *param_nh_;
  ros::Subscriber gt_sub;
  ros::Subscriber sub;
  // Components for publishing
  ros::Publisher  robot_pose_pub_;
  Eigen::Affine3d pose_;
  boost::mutex m, message_m;
  std::string gt_topic, bag_name;
  ros::Publisher map_publisher_;
  bool got_pose_target=false,got_keyboard_target=false;
  int target_node=0;
  Eigen::Affine3d pose_target;
  PlotMarker marker_;

  bool Save_submap_metadata_=false;
  std::string output_path="";
};

int main(int argc, char **argv)
{
  string file_name;
  string output_filename;
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("file-name", po::value<std::string>(&file_name)->default_value(std::string("full_map_serialization.dat")), "name of file to load containing graphMap")
      ("output-filepath", po::value<std::string>(&output_filename)->default_value(std::string("/home/submap_metadata")), "name of file to load containing graphMap");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    exit(0);
  }
  ros::init(argc, argv, "show_map");
  ros::NodeHandle param("~");
  std::cout<<"Attempt to open map: "<<file_name<<std::endl;

  ros::Rate loop_rate(10);
  OpenGraphMap t(&param, file_name);
  t.SetFilePath(output_filename);
  std::thread input_th (&OpenGraphMap::KeyboardInputThread,&t);
  t.visualize();
  sleep(1.0);
  t.visualize();
  while (param.ok()) {
    t.processFrame();
    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}



