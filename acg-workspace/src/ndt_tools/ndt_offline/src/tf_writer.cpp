/*##############################################################################################################################
### BY DANIEL ADOLFSSON
### USAGE: head to the folder containing target rosbag   "rosrun ndt_offline tf_writer --bag-file-path 2012-01-08_vel_odom.bag --data-set michigan --odom-topic /robot_odom" make sure the odometry message topic is specified by the command --odom-topic
<<<<<<< HEAD
###
*/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/message_filter.h>
#include <boost/program_options.hpp>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>
#include <ndt_offline/convertvelodynebags.h>
#include <eigen_conversions/eigen_msg.h>
#include <angles/angles.h>
#include <velodyne_msgs/VelodyneScan.h>
#include "tf_conversions/tf_kdl.h"
#include <ros/package.h>
#include "ndt_generic/sensors_utils.h"
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

using namespace std;
namespace po = boost::program_options;
bool create_odom=true,create_tf=true;
bool interactive;
double sensor_time_offset;
// This will underestimate the yaw.
class tfWriter{
public:
  tfWriter(const string &filename, const std::string &topic,const std::string &link_id, const std::string &parent){
    topic_=topic;
    odom_link_=link_id;
    parent_=parent;
    bag_in_.open(filename, rosbag::bagmode::Read);
    bag_output_.open(filename+"_edited.bag", rosbag::bagmode::Write);
    view_ = new rosbag::View(bag_in_);
    std::cout<<"Messages found :"<<view_->size()<<std::endl;
    if(view_->size()==0)
      exit(0);
    I = view_->begin();
  }
  void ConvertAll(){
    int messages_converted=0, iteration=0;
    ros::Time t_last_print=ros::Time::now();
    while (I!=view_->end()) {
      rosbag::MessageInstance const m = *I;
      if(m.getTopic()==topic_){
        if (nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>()){
          tf::tfMessage tf_msg;
          tf::Transform trans;
          ros::Time t=odom->header.stamp;

          geometry_msgs::TransformStamped msg_odom;
          tf::poseMsgToTF(odom->pose.pose,trans);
          tf::transformStampedTFToMsg(tf::StampedTransform(trans,t , parent_,odom_link_),msg_odom);
          msg_sensor_.header.stamp = t;
          tf_msg.transforms.push_back(msg_odom);
          tf_msg.transforms.push_back(msg_sensor_);

          nav_msgs::Odometry odom_corrected;
          tf::poseTFToMsg(trans,odom_corrected.pose.pose);
          odom_corrected.header.stamp=t;
          odom_corrected.header.frame_id=parent_;

          bag_output_.write(std::string("/tf"), t, tf_msg);
          bag_output_.write(m.getTopic(),t,odom_corrected);
          messages_converted++;
        }
      }
        bag_output_.write(m.getTopic(),m.getTime(),m);
        if((ros::Time::now()-t_last_print).toSec()>1.0){
          t_last_print=ros::Time::now();
          double d=((double)std::distance(view_->begin(),I))/((double)std::distance(view_->begin(),view_->end()));
          printProgress(d);
        }

      I++;
    }
    cout<<messages_converted<<" msg's with topic \""<<topic_<<"\" converted into \""<<odom_link_<<"\""<<endl;
    bag_in_.close();
    bag_output_.close();
  }
  void SetSensor(const tf::Transform &T_offset, const string &sensor_link){
    tf::transformStampedTFToMsg(tf::StampedTransform(T_offset,ros::Time::now(), odom_link_,sensor_link),msg_sensor_);
  }
  void printProgress (double percentage)
  {
      int val = (int) (percentage * 100);
      int lpad = (int) (percentage * PBWIDTH);
      int rpad = PBWIDTH - lpad;
      printf ("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
      fflush (stdout);
  }
private:
  std::string topic_;
  string odom_link_,parent_;
  rosbag::Bag bag_in_,bag_output_;
  rosbag::View *view_;
  rosbag::View::iterator I;
  geometry_msgs::TransformStamped msg_sensor_;
};

int main(int argc, char **argv){

  ros::Time::init();
  Eigen::Vector3d transl;
  Eigen::Vector3d euler;
  tf::Transform tf_sensor_pose;
  string inbag_name, dataset;
  string odom_topic;
  std::string base_link_id,gt_base_link_id,tf_world_frame,sensor_link_id;
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("debug", "additional output")
      ("bag-file-path", po::value<string>(&inbag_name)->required(), "bag file to extend with tf")
      ("odom-link", po::value<std::string>(&base_link_id)->default_value(std::string("/odom_base_link")), "desired link id for odometry")
      ("sensor-link", po::value<std::string>(&sensor_link_id)->default_value(std::string("/velodyne")), "desired link id for sensor to add as a child of odometry")
      ("odom-topic", po::value<std::string>(&odom_topic)->default_value(std::string("/robot_odom")), "topic of odometry message which will be copied to tf tree")
      ("world-link", po::value<std::string>(&tf_world_frame)->default_value(std::string("/world")), "tf world frame")
      ("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("data-set", po::value<string>(&dataset)->default_value(std::string("arla-2012")), "choose which dataset that is currently used, this option will assist with assigning the sensor pose")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }

  Eigen::Affine3d pose;
  ndt_generic::GetSensorPose(dataset,transl,euler,tf_sensor_pose,pose);
  std::cout<<"Adding sensor with link \""<<sensor_link_id<<"\" to tf (x y z)= ("<<pose.translation().transpose()<<")"<<std::endl;
  tfWriter writer(inbag_name,odom_topic,base_link_id,tf_world_frame);
  writer.SetSensor(tf_sensor_pose,sensor_link_id);
  writer.ConvertAll();


  return 0;
}
