#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "ndt_rviz/ndt_line_visual.hpp"
#include "acg_maps.hpp"
#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>

namespace AASS{
	namespace acg {
  
  ACGMapsDisplay::ACGMapsDisplay(){
    ROS_ERROR("BUILDING OBJECT");
    color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                               "Color to draw the acceleration arrows.",
                                               this, SLOT( updateColorAndAlpha() ));

    alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                               "0 is fully transparent, 1.0 is fully opaque.",
                                               this, SLOT( updateColorAndAlpha() ));

    history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                      "Number of prior measurements to display.",
                                                      this, SLOT( updateHistoryLength() ));
    history_length_property_->setMin( 1 );
    history_length_property_->setMax( 100000 );
  }
  void ACGMapsDisplay::onInitialize(){
    MFDClass::onInitialize();
  }

  ACGMapsDisplay::~ACGMapsDisplay(){
  }
  
  void ACGMapsDisplay::reset(){
    MFDClass::reset();
    visuals_.clear();
// 	all_visuals_.clear();
  }
  
  void ACGMapsDisplay::updateColorAndAlpha(){
    float alpha=alpha_property_->getFloat();
    Ogre::ColourValue color=color_property_->getOgreColor();
    for(size_t i=0;i<visuals_.size();i++){
      visuals_[i]->setColor(color.r,color.g,color.b,alpha);
    }
//     for(size_t i=0;i<all_visuals_.size();i++){
//       for(size_t j=0;j<all_visuals_[i].size();j++){
// 		all_visuals_[i][j]->setColor(color.r,color.g,color.b,alpha);
// 	  }
//     }
  }
  
void ACGMapsDisplay::updateHistoryLength()
{
	ROS_INFO_STREAM("history received: " << this->history_length_property_->getInt());
	 
}

  
  void ACGMapsDisplay::processMessage(const auto_complete_graph::ACGMaps::ConstPtr& msg ){
    ROS_INFO_STREAM(" vector MESSAGE RECIVED with history: " << this->history_length_property_->getInt() << " and deque size " << visuals_.size() );
	
// 	std::cout << "MSG detail. nb of maps: " <<msg->maps.size() << std::endl;
	
	visuals_.clear();
	processVectorMaps(msg->ndt_maps);
	
  }
  
void ACGMapsDisplay::processVectorMaps(const ndt_map::NDTVectorMapMsg& msg)
{

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
	
    for(int i = 0 ; i < msg.maps.size() ; ++i){
			
		std::cout << "For map " << i << " there is " << msg.maps[i].cells.size() << " cells " << std::endl;

//	    if(i == 0 ){
		    position.x = msg.poses[i].position.x;
		    position.y = msg.poses[i].position.y;
		    position.z = msg.poses[i].position.z;

		    orientation.x = msg.poses[i].orientation.x;
		    orientation.y = msg.poses[i].orientation.y;
		    orientation.z = msg.poses[i].orientation.z;
		    orientation.w = msg.poses[i].orientation.w;
//	    }else {
//
//		    Eigen::Affine3d pose_transformation_tmp;
//		    geometry_msgs::Pose pose_t_msg;
//		    pose_t_msg.position.x = msg.transformations[i].translation.x;
//		    pose_t_msg.position.y = msg.transformations[i].translation.y;
//		    pose_t_msg.position.z = msg.transformations[i].translation.z;
//		    pose_t_msg.orientation.x = msg.poses[i].orientation.x;
//		    pose_t_msg.orientation.y = msg.poses[i].orientation.y;
//		    pose_t_msg.orientation.z = msg.poses[i].orientation.z;
//		    pose_t_msg.orientation.w = msg.poses[i].orientation.w;
//		    tf2::fromMsg(pose_t_msg, pose_transformation_tmp);
//
//		    geometry_msgs::Pose current_pose_msg;
//		    current_pose_msg.position.x = position.x;
//		    current_pose_msg.position.y = position.y;
//		    current_pose_msg.position.z = position.z;
//		    current_pose_msg.orientation.x = orientation.x;
//		    current_pose_msg.orientation.y = orientation.y;
//		    current_pose_msg.orientation.z = orientation.z;
//		    current_pose_msg.orientation.w = orientation.w;
//
//		    Eigen::Affine3d current_pose;
//		    tf2::fromMsg(current_pose_msg, current_pose);
//			current_pose = current_pose * pose_transformation_tmp;
//
//		    geometry_msgs::Pose new_pose = tf2::toMsg(current_pose);
//
//		    position.x = new_pose.position.x;
//		    position.y = new_pose.position.y;
//		    position.z = new_pose.position.z;
//		    orientation.x = new_pose.orientation.x;
//		    orientation.y = new_pose.orientation.y;
//		    orientation.z = new_pose.orientation.z;
//		    orientation.w = new_pose.orientation.w;
//
//		    std::cout << "Pose and noisy pose. X : " << msg.poses[i].position.x << " " << new_pose.position.x << ". Y : " << msg.poses[i].position.y << " " << new_pose.position.y << ". Z : " << msg.poses[i].position.z << " " << new_pose.position.z << std::endl;
//
//	    }
		

		
		for(int itr=0;itr<msg.maps[i].cells.size(); itr++){
			
			if(msg.maps[i].cells[itr].hasGaussian_ == true){
	
				boost::shared_ptr<perception_oru::NDTLineVisual> visual;
				visual.reset(new perception_oru::NDTLineVisual(context_->getSceneManager(), scene_node_));

				if(!(msg.maps[i].x_cell_size == msg.maps[i].y_cell_size && msg.maps[i].y_cell_size == msg.maps[i].z_cell_size)){ 
					ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE"); 
					//return false;
				}
				
				visual->setCell(msg.maps[i].cells[itr], msg.maps[i].x_cell_size);
				visual->setFramePosition(position);
				visual->setFrameOrientation(orientation);
				float alpha = alpha_property_->getFloat();
				Ogre::ColourValue color=color_property_->getOgreColor();
				visual->setColor(color.r,color.g,color.b,alpha);
				visuals_.push_back(visual);
			}
				
		}
	}
    
    
  }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(AASS::acg::ACGMapsDisplay,rviz::Display)

