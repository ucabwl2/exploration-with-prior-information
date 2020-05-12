#ifndef GRAPHMAPCONVERSION_H_10122017
#define GRAPHMAPCONVERSION_H_10122017

#include <time.h>

#include "graph_map/ndt/ndt_map_type.h"
#include "graph_map/factor.h"
#include "graph_map/graph_map.h"
#include "graph_map/NodeMsg.h"
#include "graph_map/FactorMsg.h"
#include "graph_map/GraphMapMsg.h"
#include "eigen_conversions/eigen_msg.h"
#include "ndt_map/ndt_conversions.h"
#include "ndt_map/NDTVectorMapMsg.h"

namespace perception_oru{
namespace graph_map{
	
	void nodeToMsg(MapNode& node, ::graph_map::NodeMsg& msg, const std::string& frame){
// 		std::cout << "NODE " << node.GetId() << std::endl;
		msg.id.data = node.GetId();
// 		std::cout << "NODE" << std::endl;
		tf::poseEigenToMsg(node.GetPose(), msg.pose);
// 		std::cout << "NODE" << std::endl;
		auto map = node.GetMap();
// 		std::cout << "NODE " << map.get() << std::endl;
		NDTMapType* node_ptr = dynamic_cast<NDTMapType*>(map.get());
// 		std::cout << "NODE : " << node_ptr << std::endl;
		assert(node_ptr != NULL);
		perception_oru::NDTMap* map_ptr = node_ptr->GetNDTMap();

//		assert(map_ptr->getAllCells().size() != 0);

		bool good = perception_oru::toMessage(map_ptr, msg.ndt_map, frame);

//		assert(map_ptr->getAllCells().size() == msg.ndt_map.cells.size());
// 		std::cout << "NODE END" << std::endl;


	}
	
	//Not const factor because GteNodes return pointers
	void factorToMsg(perception_oru::graph_map::factor& factor, ::graph_map::FactorMsg& msg){

// 		std::cout << "Factor" << std::endl;
		tf::poseEigenToMsg(factor.GetDiff(), msg.diff);
// 		std::cout << "Factor" << factor.GetDiff().matrix() << std::endl;
//		exit(0);
		tf::matrixEigenToMsg(factor.GetCovariance(), msg.covariance);
// 		std::cout << "Factor" << std::endl;

		NodePtr prev = NULL;
		NodePtr next = NULL;
		factor.GetNodes(prev, next);
// 		std::cout << "Factor" << prev << " " << next << std::endl;
		assert(prev != NULL);
		assert(next != NULL);
// 		std::cout << "Factor" << std::endl;
		msg.prev.data = prev->GetId();
// 		std::cout << "Factor" << std::endl;
		msg.next.data = next->GetId();
// 		std::cout << "Factor END" << std::endl;
	}
	
	//Not const GraphMap because of the factors
	void graphMapToMsg(const GraphMap& graphmap, ::graph_map::GraphMapMsg& msg, const std::string& frame){

		msg.header.frame_id = frame;
		msg.header.stamp = ros::Time::now();

//		msg.compound_radius.data = graphmap.CompoundRadius();
//		msg.interchange_radius.data = graphmap.InterchangeRadius();
//		msg.min_keyframe_dist.data = graphmap.MinKeyframeDist();
//		msg.min_keyframe_rot_deg.data= graphmap.MinKeyframeRotDeg();

//		msg.use_keyframe.data = graphmap.UseKeyframe();
//		msg.use_submap.data = graphmap.UseSubmaps();

		auto nodes = graphmap.GetMapNodes();
		auto factors = graphmap.GetFactors();

//		std::cout << "Node" << std::endl;

		for (auto it = nodes.begin(); it != nodes.end(); ++it){
			::graph_map::NodeMsg nodemsg;
			nodeToMsg(**it, nodemsg, frame);
			msg.nodes.push_back(nodemsg);
		}

//		std::cout << "Factors" << std::endl;

		for (auto it = factors.begin(); it != factors.end(); ++it){
			::graph_map::FactorMsg factormsg;
			factorToMsg(**it, factormsg);
			msg.factors.push_back(factormsg);
		}


	}





	void graphMapToVectorMap(const GraphMap& graphmap, ndt_map::NDTVectorMapMsg& vector_maps, const std::string& frame) {

		vector_maps.header.frame_id = frame;
		vector_maps.header.stamp = ros::Time::now();

		auto nodes = graphmap.GetMapNodes();
		for (auto it = nodes.begin(); it != nodes.end(); ++it){

//			msg.id.data = node.GetId();
// 		std::cout << "NODE" << std::endl;
			geometry_msgs::Pose msg_pose;
			tf::poseEigenToMsg( (*it)->GetPose(), msg_pose);

			vector_maps.poses.push_back(msg_pose);

// 		std::cout << "NODE" << std::endl;
			auto map = (*it)->GetMap();
// 		std::cout << "NODE " << map.get() << std::endl;
			NDTMapType* node_ptr = dynamic_cast<NDTMapType*>(map.get());
// 		std::cout << "NODE : " << node_ptr << std::endl;
			assert(node_ptr != NULL);
			perception_oru::NDTMap* map_ptr = node_ptr->GetNDTMap();
			ndt_map::NDTMapMsg ndt_map_msg;
			bool good = perception_oru::toMessage(map_ptr, ndt_map_msg, frame);

			vector_maps.maps.push_back(ndt_map_msg);

//			std::cout << "At node " << (*it)->GetId() << std::endl;
			assert(map_ptr->getAllCells().size() != 0);

		}



	}
	
}
}

#endif