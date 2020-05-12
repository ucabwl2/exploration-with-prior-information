namespace perception_oru{
namespace graph_map{


template<class PointT>
bool GraphMapFuser::ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion){
  Eigen::MatrixXd cov=Eigen::MatrixXd::Identity(6,6);
  return ProcessFrame(cloud,Tnow,Tmotion,cov);
}
/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */
template<class PointT>
bool GraphMapFuser::ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion, Eigen::MatrixXd &Tcov){
  if(use_scanmatching)
    return scanmatching(cloud,Tnow,Tmotion);

  if(!initialized_){
    cerr<<"fuser not initialized"<<endl;
    return false;
  }

  static bool map_node_changed=false;
  bool registration_succesfull=true;
  bool map_node_created=false, fuse_this_frame=false;
  Eigen::MatrixXd cov=Tcov;
  Tnow=Tnow*Tmotion;

  fuse_this_frame=FuseFrame(Tnow,Tmotion) | map_node_changed;//fuse frame based on distance traveled, also fuse this frame if just changed to precious map*/
  //cout<<"fuse?: "<<fuse_this_frame<<endl;
  Matrix6d motion_cov = motion_model_3d_.getCovMatrix(Tmotion);
  pcl::PointCloud <PointT> tmp=cloud;
  transformPointCloudInPlace(sensorPose_, cloud);

  //Transform cloud into robot frame before registrating
  if(first_map_initialised) {
    if (fuse_this_frame || map_node_changed) {
      if (graph_map_->GetCurrentNode()->GetMap()->Initialized()) {
        graph_map_->WorldToLocalMapFrame(Tnow);
        registration_succesfull = registrator_->RegisterScan(graph_map_->GetCurrentNode()->GetMap(), Tnow, cloud,
                                                             cov);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration

    Eigen::Affine3d Tsensor_pose;
        graph_map_->LocalToWorldMapFrame(Tnow);
      }
      Eigen::Affine3d Tsensor_pose;
      Tsensor_pose = Tnow * sensorPose_;

      graph_map_->AutomaticMapInterchange(Tsensor_pose, motion_cov, map_node_changed, map_node_created);
    }
  }

  if( ( fuse_this_frame && registration_succesfull) ||(fuse_this_frame && gt_mapping_enabled_) || map_node_created || !first_map_initialised){//if changed to previously created map node, start by map_registration
    first_map_initialised = true;
    ROS_DEBUG_STREAM("Init new map node" );
    if(multiple_map_update && graph_map_->Size()>1)
      UpdateMultipleMaps(cloud,Tnow);
    else
      UpdateSingleMap(cloud,Tnow);


  if ( save_merged_cloud_)
      pcl::io::savePCDFileASCII ("cloud"+ndt_generic::toString(nr_frames_)+".pcd", tmp);

  nr_frames_++;
  return true;
  }
  else return false;

}
template<class PointT>
bool GraphMapFuser::ProcessFrameStaticGlobalMap(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion){

  static bool fuse_prev=true;
  bool fuse_frame=true;
  static pcl::PointCloud<PointT> merged_cloud;
  static unsigned int merged_clouds_count=0;
  bool registration_succesfull=true;
  bool updated=false;
  static int stand_still_frames=0;
  static int nb_frames = 0;
  if(!initialized_){
    cerr<<"fuser not initialized"<<endl;
    return false;
  }

  nr_frames_++;
  Tnow=Tnow*Tmotion;

  fuse_frame=FuseFrame(Tnow,Tmotion);//return true if vehicle standing still
  cout<<"fuse?: "<<fuse_frame<<endl;
  if(fuse_frame==true &&fuse_prev==false) {//stannade precis till
    transformPointCloudInPlace(sensorPose_, cloud);
    merged_cloud+=cloud;
    merged_clouds_count++;
  }
  else if(fuse_frame==true &&fuse_prev==true){//been standing still since last frame
    transformPointCloudInPlace(sensorPose_, cloud);
    merged_cloud+=cloud;
    merged_clouds_count++;
  }
  else if(fuse_frame==false &&fuse_prev==true){//started moving, should register towards previous map here

      Eigen::Affine3d Tinit=Tnow;
      if(graph_map_->GetCurrentNode()->GetMap()->Initialized()){
        graph_map_->WorldToLocalMapFrame(Tinit);
        registration_succesfull = registrator_->RegisterScan(graph_map_->GetCurrentNode()->GetMap(),Tinit,merged_cloud);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
        graph_map_->LocalToWorldMapFrame(Tinit);
        //Tinit är skattning(väldigt bra skattning), merged_cloud är alla punkter som skall användas för att uppdatera kartan. Tmotion är skattad rörelse
      }
      if(registration_succesfull ) {
        Tnow = Tinit;
        updated = true;
        UpdateSingleMap(merged_cloud, Tnow);
        pose_last_fuse_ = Tnow;

        if (save_merged_cloud_) {
          Eigen::Affine3d sensorPose_inv = sensorPose_.inverse();
          transformPointCloudInPlace(sensorPose_inv, merged_cloud);
          pcl::io::savePCDFileASCII("merged_cloud" + ndt_generic::toString(nb_frames++) + ".pcd", merged_cloud);
        }
      }
      else
        ROS_ERROR_STREAM("REG error");

      merged_cloud.clear();
      merged_clouds_count=0;
  }
  fuse_prev=fuse_frame;

  return updated;
}
template<class PointT>
bool GraphMapFuser::ProcessFrameStatic(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion){

  static bool fuse_prev=true;
  bool fuse_frame=true;
  static pcl::PointCloud<PointT> merged_cloud;
  static unsigned int merged_clouds_count=0;
  bool registration_succesfull=false;
  bool updated=false;
  if(!initialized_){
    cerr<<"fuser not initialized"<<endl;
    return false;
  }

  nr_frames_++;
  Tnow=Tnow*Tmotion;

  fuse_frame=FuseFrame(Tnow,Tmotion);//return true if vehicle standing still
  if(fuse_frame==true &&fuse_prev==false) {//stannade precis till
    transformPointCloudInPlace(sensorPose_, cloud);
   // graph_map_->AddMapNode(graph_map_->GetCurrentNodePose().inverse()*Tnow*sensorPose_); //add new node of current node pose

    merged_cloud+=cloud;
    merged_clouds_count++;
  }
  else if(fuse_frame==true &&fuse_prev==true){//been standing still since last frame
    transformPointCloudInPlace(sensorPose_, cloud);
    merged_cloud+=cloud;
    merged_clouds_count++;
  }
  else if(fuse_frame==false &&fuse_prev==true){//started moving, should register towards previous map here
    UpdateSingleMap(merged_cloud,Tnow);
    pose_last_fuse_=Tnow;
    merged_cloud.clear();
    merged_clouds_count=0;

    if(graph_map_->Size()>=2 ){

      Eigen::Affine3d diff=graph_map_->GetPreviousNodePose().inverse()*graph_map_->GetCurrentNodePose();

      double score;
      ROS_DEBUG_STREAM("register");
      NDTD2DRegTypePtr ndt_reg=boost::dynamic_pointer_cast<NDTD2DRegType>(registrator_);
      bool registration_succesfull; //register here

      if(registration_succesfull ){

        updated=true;
        ROS_DEBUG_STREAM("REG success");
        Tnow=graph_map_->GetPreviousNodePose()*diff*sensorPose_.inverse();
       // NDTMapPtr prev_node = boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetPreviousNode()->GetMap());
      //  GraphPlot::SendGlobalMapToRviz(prev_node->GetNDTMap(),1,graph_map_->GetPreviousNodePose());
        graph_map_->UpdateLink(graph_map_->GetPreviousNode(),graph_map_->GetCurrentNode(),graph_map_->GetPreviousNodePose().inverse()*graph_map_->GetCurrentNodePose());
        //GraphPlot::PlotMap(graph_map_->GetCurrentNode()->GetMap(),1,graph_map_->GetCurrentNodePose());
      }
      else
        ROS_DEBUG_STREAM("REG error");
    }

  }
  else if(fuse_frame==false &&fuse_prev==false){ //moving, do nothing
  }

    fuse_prev=fuse_frame;

  return updated;
}

template<class PointT>
void GraphMapFuser::UpdateSingleMap(pcl::PointCloud<PointT> &cloud,Eigen::Affine3d &Tnow){ //specified in the global frame
  ROS_DEBUG_STREAM("Update");
  graph_map_->WorldToLocalMapFrame(Tnow);
  transformPointCloudInPlace(Tnow, cloud);
  graph_map_->GetCurrentNode()->updateMap/*<PointT>*/(Tnow*sensorPose_,cloud);//Update map, provided transform is the pose of the sensor in the world which is where the scan was taken from
  graph_map_->LocalToWorldMapFrame(Tnow);
  pose_last_fuse_=Tnow;
}
template<class PointT>
void GraphMapFuser::UpdateMultipleMaps(pcl::PointCloud<PointT> &cloud,Eigen::Affine3d &Tnow){//specified in the global frame

  std::vector<MapNodePtr> closest_nodes = graph_map_->GetClosestNodes(Tnow*sensorPose_,1.0,true);
  if(closest_nodes.size()==0 )
    ROS_DEBUG_STREAM("close nodes There should always be a node within interchange radius");
  else if(closest_nodes.size()==1){
    UpdateSingleMap(cloud,Tnow);
    ROS_DEBUG_STREAM("close nodes, one");
  }
  else
  {
    for(int i=0;i<closest_nodes.size();i++){
      pcl::PointCloud<PointT> cloud_map_i;
      copyPointCloud(cloud_map_i,cloud);
      MapNodePtr map_i=closest_nodes[i];
      Eigen::Affine3d Tinit=Tnow;
      graph_map_->WorldToLocalMapFrame(Tinit,map_i);
      transformPointCloudInPlace(Tinit, cloud_map_i);
      map_i->updateMap/*<PointT>*/(Tinit*sensorPose_,cloud_map_i);//Update map, provided transform is the pose of the sensor in the world which is where the scan was taken from
      ROS_DEBUG_STREAM("update node="<<map_i->GetPose().translation());
    }
    pose_last_fuse_=Tnow;
  }
}

template<class PointT>
bool GraphMapFuser::scanmatching(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow,  Eigen::Affine3d &Tmotion){
  if(!initialized_){
    cerr<<"fuser not initialized"<<endl;
    return false;
  }

  Eigen::Affine3d Tinit=Tnow;
  bool registration_succesfull = registrator_->RegisterScan(graph_map_->GetCurrentNode()->GetMap(),Tinit,cloud);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration
  if(registration_succesfull){

    bool fuse_this_frame=KeyFrameBasedFuse(Tinit);//fuse frame based on distance traveled, also fuse this frame if just changed to precious map*/
    if( fuse_this_frame){
      Eigen::Affine3d Tmotion=Tnow.inverse()*Tinit;
      graph_map_->WorldToLocalMapFrame(Tnow);
      graph_map_->AddMapNode(Tmotion);
      transformPointCloudInPlace(Tnow, cloud);
      graph_map_->GetCurrentNode()->updateMap/*<PointT>*/(Tnow*sensorPose_,cloud);//Update map, provided transform is the pose of the sensor in the world which is where the scan was taken from
      graph_map_->LocalToWorldMapFrame(Tnow);
      pose_last_fuse_=Tnow;
      return true;
    }
  }
  return false;


}



}
}
