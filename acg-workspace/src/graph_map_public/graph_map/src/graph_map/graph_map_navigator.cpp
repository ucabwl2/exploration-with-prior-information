#include "graph_map/graph_map_navigator.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::GraphMapNavigator)
namespace perception_oru{
  namespace graph_map{
  GraphMapNavigator::GraphMapNavigator(const Eigen::Affine3d &nodepose, const MapParamPtr &mapparam, const GraphMapParamPtr graphparam): GraphMap(nodepose, mapparam ,graphparam){
    if(GraphMapNavigatorParamPtr par=boost::dynamic_pointer_cast<GraphMapNavigatorParam> (graphparam)){
      use_submap_=par->use_submap;
      interchange_radius_=par->interchange_radius;
      compound_radius_=par->compound_radius;
      use_keyframe_=par->use_keyframe;
      min_keyframe_dist_=par->min_keyframe_dist;
      min_keyframe_rot_deg_=par->min_keyframe_rot_deg;
      map_nodes_grid_.Initialize(Eigen::Vector3d(interchange_radius_,interchange_radius_,interchange_radius_),nodepose.translation());
      map_nodes_grid_.SetVal(currentNode_,GetCurrentNodePose().translation());
      map_switch_method_=par->map_switch_method;
      n_search_=par->n_search;
      alpha_=par->alpha;
      k_=0;//par->interchange_radius/M_PI; //if 3d movement, use 3*sqrt(M_PI) instead of PI
      Tsensor_=par->Tsensor;
      ROS_INFO_STREAM("Created GraphMapNavigator");
    }
    else{
      std::cerr<<"Cannot create GraphMapNavigator"<<std::endl;
      exit(0);
    }

  }

  GraphMapNavigator::GraphMapNavigator(){}

  void GraphMapNavigator::WorldToLocalMapFrame(Eigen::Affine3d &pose,MapNodePtr frame){
    if(frame==NULL)
      pose=GetCurrentNodePose().inverse()*pose;
    else
      pose=frame->GetPose().inverse()*pose;
  }
  void GraphMapNavigator::LocalToWorldMapFrame(Eigen::Affine3d &pose,MapNodePtr frame){
    if(frame==NULL)
      pose=GetCurrentNodePose()*pose;
    else
      pose=frame->GetPose()*pose;
  }

  bool GraphMapNavigator::SwitchToClosestMapNode(const Affine3d &Tnow, double max_distance , pcl::PointCloud<pcl::PointXYZ> *cloud){

    double min_dist=DBL_MAX;
    MapNodePtr closest_map_node=NULL;

    if(map_switch_method_==node_position ){
      ROS_DEBUG_STREAM( "SW: node position");
      closest_map_node=GetClosestMapNode(Tnow);

      if(closest_map_node->WithinRadius(Tnow,max_distance))
        return SwitchToMapNode(closest_map_node);
      else
        return false;
    }
    else if(map_switch_method_==grid){
      ROS_DEBUG_STREAM( "SW: grid");
      map_nodes_grid_.GetVal(closest_map_node,Tnow.translation());
      return SwitchToMapNode(closest_map_node);
    }
    else if(map_switch_method_==overlap){
      ROS_DEBUG_STREAM( "SW: overlap");
      closest_map_node=GetMapByOverlap(Tnow,cloud);
    }
    else {
      ROS_DEBUG_STREAM( "SW: observation_history");
      closest_map_node=GetMapNodeByObservationHistory(Tnow,n_search_);
      ROS_DEBUG_STREAM( "switch to this node");
      return SwitchToMapNode(closest_map_node);
    }

  }
  std::vector<MapNodePtr> GraphMapNavigator::GetClosestNodes(const Eigen::Affine3d &Tnow, double max_distance, bool factor_interchange){
    double distance=max_distance;
    if(factor_interchange)
      distance=interchange_radius_*max_distance;

    std::vector<MapNodePtr> close_nodes;
    bool centroid=(map_switch_method_==mean_observation)?true:false;

    for(std::vector<MapNodePtr>::iterator itr_node = map_nodes_.begin(); itr_node != map_nodes_.end(); ++itr_node) { //loop thorugh all mapnodes and list the one closer than max_distance
      if( ndt_generic::GetDistance((*itr_node)->GetMapPose(false),Tnow,k_,alpha_)<distance)
        close_nodes.push_back(*itr_node);
    }
    return close_nodes;
  }
  MapNodePtr GraphMapNavigator::GetMapByOverlap(const Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> *cloud){

    if(cloud==NULL){
      ROS_DEBUG_STREAM( "start with closest map node");
      return GetClosestMapNode(Tnow);
    }
    ROS_DEBUG_STREAM( "Get actually node overlap");


    double max_distance=interchange_radius_*2.0;
    ROS_DEBUG_STREAM( "max d="<<max_distance);
    std::vector<MapNodePtr> close_nodes=GetClosestNodes(Tnow,max_distance);
    unsigned int overlap=0, max_overlap=0;
    MapNodePtr most_overlapping_map;
    for(std::vector<MapNodePtr>::iterator itr_node = close_nodes.begin(); itr_node != close_nodes.end(); ++itr_node) { //loop thorugh all close nodes and find the observation closest to Tnow among all map nodes
      pcl::PointCloud<pcl::PointXYZ> tmp_cloud=*cloud;
      Eigen::Affine3d T=Tnow;
      WorldToLocalMapFrame(T,(*itr_node));
      transformPointCloudInPlace(T,tmp_cloud);
      NDTMap *ndtmap=boost::dynamic_pointer_cast<NDTMapType>((*itr_node)->GetMap())->GetNDTMap();
      overlap=ndtmap->getOverlap(tmp_cloud);
      Eigen::Affine3d node_pose=(*itr_node)->GetPose();
      double d=ndt_generic::GetDistance(node_pose.translation(),Tnow.translation());
      ROS_DEBUG_STREAM( "overlap with node:"<<itr_node-close_nodes.begin()<<",="<<overlap<<", diff="<<d);

      if(overlap>max_overlap){
        max_overlap=overlap;
        most_overlapping_map=(*itr_node);
      }
    }
    return most_overlapping_map;
  }
  MapNodePtr GraphMapNavigator::GetMapNodeByObservationHistory(const Eigen::Affine3d &Tnow){

    double max_distance=interchange_radius_*2.0;
    std::vector<MapNodePtr> close_nodes=GetClosestNodes(Tnow,max_distance);
    MapNodePtr map_closest_observation=NULL;
    double min_distance=DBL_MAX;
    for(std::vector<MapNodePtr>::iterator itr_node = close_nodes.begin(); itr_node != close_nodes.end(); ++itr_node) { //loop thorugh all close nodes and find the observation closest to Tnow among all map nodes
      ndt_generic::Affine3dSTLVek obs_vec =(*itr_node)->GetObservationVector();
      double distance = ndt_generic::SearchForClosestElement(Tnow,obs_vec,k_,alpha_);
      if(distance<min_distance){
        min_distance=distance;
        map_closest_observation=(*itr_node);
      }
    }
    return map_closest_observation;
  }
  MapNodePtr GraphMapNavigator::GetMapNodeByObservationHistory(const Eigen::Affine3d &Tnow, unsigned int n_search){

    double max_distance=interchange_radius_*2.0;
    std::vector<MapNodePtr> close_nodes=GetClosestNodes(Tnow,max_distance); //get all nodes within a distance
    std::vector< std::pair<double,MapNodePtr> >observation_distance;

    for(std::vector<MapNodePtr>::iterator itr_node = close_nodes.begin(); itr_node != close_nodes.end(); ++itr_node) { //loop thorugh all close nodes and find the observation closest to Tnow among all map nodes
      ndt_generic::Affine3dSTLVek obs_vec =(*itr_node)->GetObservationVector();//get all observation locations in submap
      double distance;
      for(int i=0;i<obs_vec.size();i++){
        distance=ndt_generic::GetDistance(obs_vec[i],Tnow,k_,alpha_);//calculate distance from robot sensor to each observation
        observation_distance.push_back(std::make_pair(distance,*itr_node)); // create toubles of (distance , map-node)
      }
    }
    //Sort vector by distance to observation
    std::sort(observation_distance.begin(),observation_distance.end()); //sort vector of toubles

    std::vector<unsigned int>obs_histogram(Size(),0);//histogram over nodes for n closest observations
    for(int i=0;i<observation_distance.size() && i<n_search*close_nodes.size();i++){ //create histogram of map-node occurance, data for histogram is limited by close nodes*n_search
      unsigned int node_id=observation_distance[i].second->GetId();
      obs_histogram[node_id]=obs_histogram[node_id]+1;
    }
    unsigned int max_index=distance(obs_histogram.begin(), max_element(obs_histogram.begin(),obs_histogram.end())); //select most occuring node

    return GetNode(max_index);
  }

  MapNodePtr GraphMapNavigator::GetClosestMapNode(const Eigen::Affine3d &Tnow,const bool use_observation_centroid){
    MapNodePtr closest_map_node=NULL;
    double min_dist=DBL_MAX;
    for(std::vector<MapNodePtr>::iterator itr_node = map_nodes_.begin(); itr_node != map_nodes_.end(); ++itr_node) { //loop thorugh all existing nodes, select the closest one

      double distance= ndt_generic::GetDistance((*itr_node)->GetMapPose(use_observation_centroid),Tnow,k_,alpha_);
      if(distance<min_dist){
        closest_map_node=*itr_node;
        min_dist=distance;
      }
    }
    return closest_map_node;
  }

  bool GraphMapNavigator::SwitchToMapNode( MapNodePtr new_node){

    if(new_node!=NULL && currentNode_!=new_node ){
      prevNode_=currentNode_;
      currentNode_=new_node;
      ROS_DEBUG_STREAM( "Switched to node: "<<currentNode_->GetPose().translation().transpose());
      return true;
    }
    else
      return false;
  }

  bool GraphMapNavigator::AutomaticMapInterchange(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node){

    ROS_DEBUG_STREAM(  "Auto map change" );
    created_map_node=false;
    changed_map_node=false;
    static Matrix6d pose_covar=unit_covar;
    if(use_submap_){
      if(map_switch_method_==grid){
        return TransitionGrid(Tnow,cov_incr,changed_map_node,created_map_node);
      }
      else if(map_switch_method_==node_position_esg)
        return TransitionESG(Tnow,cov_incr,changed_map_node,created_map_node);
      else{
        //Default
        return TransitionSG(Tnow,cov_incr,changed_map_node,created_map_node);
      }

    }
    else
      return false;

  }
  bool GraphMapNavigator::TransitionGrid(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node){
    MapNodePtr closest_map_node;
    changed_map_node=false;
    created_map_node=false;
    bool found_node=map_nodes_grid_.GetVal(closest_map_node,Tnow.translation());
    if(found_node){
      if( SwitchToMapNode(closest_map_node)){
        changed_map_node=true;
        return true;
      }
      else
        return false;
    }
    else{

      AddMapNode(GetCurrentNodePose().inverse()*Tnow,cov_incr); //if no node already exists, create a new node
      prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(),prevNode_->GetPose(),currentNode_->GetPose(),compound_radius_);
      created_map_node=true;
      return true;
    }

  }

  bool GraphMapNavigator::TransitionSG(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node){


    //if(! currentNode_->WithinRadius(Tnow,interchange_radius_,use_centroid)){ //No longer within radius of node
    if(changed_map_node=SwitchToClosestMapNode(Tnow,interchange_radius_)){
      ROS_DEBUG_STREAM( "switched to node="<<currentNode_->GetPose().translation() );
    }
    double distance= ndt_generic::GetDistance(GetCurrentNodePose(),Tnow,k_,alpha_);

    ROS_DEBUG_STREAM( "distance to closest node=" << distance << " > " << interchange_radius_ << " for node id " << currentNode_->GetId() );
    if(  distance>interchange_radius_){
      ROS_DEBUG_STREAM( "No node was found, will create a new map pose.");
      ROS_DEBUG_STREAM(  "from " << GetCurrentNodePose().matrix() << "\n  to " << Tnow.matrix() );
      ROS_DEBUG_STREAM(  "Hence " << (GetCurrentNodePose().inverse()*Tnow).matrix() );
      ROS_DEBUG_STREAM(  "translation: " << (GetCurrentNodePose().inverse()*Tnow).translation() );
      ROS_DEBUG_STREAM(  "rotation: " << (GetCurrentNodePose().inverse()*Tnow).rotation().matrix() );
      ROS_DEBUG_STREAM(  "Hsould be tnow " << ( GetCurrentNodePose() * (GetCurrentNodePose().inverse()*Tnow) ).matrix() );
//	    exit(0);
//	    int wait = 0;
//	    std::cin >> wait ;

      AddMapNode(GetCurrentNodePose().inverse()*Tnow,cov_incr); //if no node already exists, create a new node
      prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(),prevNode_->GetPose(),currentNode_->GetPose(),compound_radius_);
      created_map_node=true;
    }
    return created_map_node || changed_map_node;
  }
  bool GraphMapNavigator::TransitionESG(const Eigen::Affine3d &Tnow, const Matrix6d &cov_incr, bool &changed_map_node, bool &created_map_node){
    created_map_node = changed_map_node = false;
    if(! currentNode_->WithinRadius(Tnow,interchange_radius_)){ //No longer within radius of node
//    bool use_centroid=(map_switch_method_==node_position_esg)?false:true;
//    if(! currentNode_->WithinRadius(Tnow,interchange_radius_,use_centroid)){ //No longer within radius of node
      ROS_DEBUG_STREAM("No node was found, will create a new map node.");
      AddMapNode(GetCurrentNodePose().inverse()*Tnow,cov_incr);
      prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(),prevNode_->GetPose(),currentNode_->GetPose(),compound_radius_);
      created_map_node = true;

    }

    ROS_DEBUG_STREAM( "Auto map change done" );
    return created_map_node || changed_map_node;
  }
  void GraphMapNavigator::UpdateGraph(const Eigen::Affine3d &pose, pcl::PointCloud<pcl::PointXYZ> &cloud){
    GetCurrentNode()->updateMap(pose,cloud);  }

  std::string GraphMapNavigator::ToString(){
    std::stringstream ss;
    ss <<"GraphMapNavigator:\n"<<GraphMap::ToString();
    ss <<"Interchange: "<<interchange_radius_<<std::endl;
    ss <<"alpha distance: "<<alpha_<<std::endl;
    ss <<"k distance: "<<k_<<std::endl;
    ss <<"Compound_radius: "<<compound_radius_<<std::endl;
    ss <<"Use keyframe: "<<std::boolalpha<<use_keyframe_<<std::endl;
    if(use_keyframe_){
      ss <<"Keyframe distance [m]: "<<min_keyframe_dist_<<std::endl;
      ss <<"Keyframe angular distance [deg]: "<<min_keyframe_rot_deg_<<std::endl;
    }
    ss <<"Switch method: "<<GraphMapNavigatorParam::SwitchMethod2String(map_switch_method_)<<std::endl;
    return ss.str();
  }
  MapSwitchingMethod GraphMapNavigatorParam::String2SwitchMethod(const std::string &switch_method){

    if(switch_method=="mean_observation")
      return mean_observation;
    else if(switch_method=="closest_observation")
      return closest_observation;
    else if(switch_method=="grid")
      return grid;
    else if(switch_method=="overlap")
      return overlap;
    else if(switch_method=="node_position_esg")
      return node_position_esg;
    else
      return node_position;

  }
  std::string GraphMapNavigatorParam::SwitchMethod2String(const MapSwitchingMethod &switch_method){
    if(switch_method==mean_observation)
      return std::string("mean_observation");
    else if(switch_method==closest_observation)
      return std::string("closest_observation");
    else if(switch_method==grid)
      return std::string("grid");
    else if(switch_method==node_position_esg)
      return std::string("node_position_esg");
    else if(switch_method==overlap)
      return std::string("overlap");
    else
      return std::string("node_position");
  }


  void GraphMapNavigatorParam::GetParametersFromRos(){
    ros::NodeHandle nh("~");
    nh.param("use_submap",use_submap,false);
    nh.param("interchange_radius",interchange_radius,10.0);
    nh.param("compound_radius",compound_radius,0.0);
    nh.param("use_keyframe",use_keyframe,true);
    nh.param("min_keyframe_dist",min_keyframe_dist,0.5);
    nh.param("min_keyframe_rot_deg",min_keyframe_rot_deg,5.0);
    std::string sw_method;
    nh.param<string>("map_switching_method",sw_method,"node_position");
    map_switch_method=String2SwitchMethod(sw_method);

  }
  bool LoadGraphMap(const std::string &file_name,  GraphMapNavigatorPtr &ptr){
    if(file_name==""){
      cout<<"No map file path provided"<<endl;
      exit(0);
    }
    ifstream f(file_name.c_str());
    if(!f.good())  {
      cout<<"The file  \""<<file_name<<"\" does not exist."<<endl;
      return false;
    }

    try{
      cout<<"Loading map at path: \n"<<file_name<<endl;
      std::ifstream ifs(file_name);
      boost::archive::text_iarchive ia(ifs);
      ia & ptr;
      cout<<"Map loaded"<<endl;
    }
    catch(const std::exception& e){
      std::cerr<<"Error loading map at path:\n"<<file_name<<std::endl;
      cerr<<e.what()<<endl;
      return false;
    }
    if(ptr==NULL){
      std::cerr<<"ERROR LOADING NDT MAP FROM FILE"<<std::endl;
      return false;
    }
    return true;
  }
  void SaveObservationVector(const std::string &file_name, GraphMapNavigatorPtr graph_map){
    std::ofstream observations_file;
    std::string name =file_name + std::string("_observation.txt");
    observations_file.open(name.c_str());
    if (observations_file.is_open()  )
    {
      for(std::vector<MapNodePtr>::iterator itr_node = graph_map->begin(); itr_node != graph_map->end(); ++itr_node) { //loop thorugh all close nodes and find the observation closest to Tnow among all map nodes
        ndt_generic::Affine3dSTLVek obs_vec= (*itr_node)->GetObservationVector();
        for(int i=0;i<obs_vec.size();i++){
          observations_file<<obs_vec[i].translation()(0)<<" "<<obs_vec[i].translation()(1)<<" "<<obs_vec[i].translation()(2)<<" "<<std::distance(graph_map->begin(), itr_node)<<std::endl;
          observations_file.flush();
        }
      }
      observations_file.close();
    }
    else{
      std::cout<<"Error creating evaluation output files at path:"<<std::endl;
      std::cout<<name<<std::endl;
      exit(0);
    }
  }

  }
}
