#include "graph_localization/mcl_ndt/mcl_ndt.h"
#include <graph_map/ndt_dl/point_curv.h>
namespace perception_oru{
namespace graph_localization{

MCLNDTType::MCLNDTType(LocalisationParamPtr param):LocalizationType(param){
  if(MCLNDTParamPtr mclParam=boost::dynamic_pointer_cast<MCLNDTParam>(param)){
    NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentNode()->GetMap());
    if (current_map != NULL) {
      resolution=boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentNode()->GetMap())->GetResolution();
    }
    else {
      ROS_INFO_STREAM("MCL-NDT is only valid for NDT maps");
      exit(0);
    }
    counter=0;
    forceSIR=mclParam->forceSIR;
    motion_model_=mclParam->motion_model;
    n_particles_=mclParam->n_particles;
    SIR_varP_threshold=mclParam->SIR_varP_threshold;
    z_filter_min=mclParam->z_filter_min;
    score_cell_weight=mclParam->score_cell_weight;
    initialized_ = false;
    forceSIR = mclParam->forceSIR;
    resolution_sensor=resolution;

    time_t t;
    srand(time(&t));
    ros::NodeHandle nh("~");
    part_pub=nh.advertise<geometry_msgs::PoseArray>("pose_particles",100);

  }
  else{
    std::cerr<<"Cannot create MCLNDType. Illegal type for parameter param"<<endl;
    exit(0);
  }
}

NDTMap* MCLNDTType::GetCurrentNodeNDTMap() {
  // Currently exist two different maps of NDT, NDT and NDTDL, find out which we have in the graph and return it...
  {
    NDTMapPtr p = boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap());
    if (p != NULL) {
      return p->GetNDTMap();
    }
  }

  {
    NDTDLMapPtr p = boost::dynamic_pointer_cast< NDTDLMapType >(graph_map_->GetCurrentNode()->GetMap());
    return p->GetNDTMapFlat();
  }
  return NULL;
}

void MCLNDTType::Subsample(std::vector<perception_oru::NDTCell*> &input, std::vector<perception_oru::NDTCell*> &output){
  if(subsample_level_ != 1) {
    for(int i=0; i<input.size(); ++i) {
      double p = ((double)rand())/RAND_MAX;
      if(p < subsample_level_) {
        output.push_back(input[i]);
      } else {
        delete input[i];
      }
    }
  } else {
    output = input;
  }
}

void MCLNDTType::AssignParticleScore(std::vector<perception_oru::NDTCell*> ndts){
  int Nn = 0;
  double t_pseudo = getDoubleTime();
#pragma omp parallel num_threads(12)
  {
#pragma omp for
    for(int i=0;i<pf.size();i++){
      Eigen::Affine3d T = pf.pcloud[i].GetAsAffine();

      double score=1;

      if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
      Nn = ndts.size();

      for(int n=0;n<ndts.size();n++){
        Eigen::Vector3d m = T*ndts[n]->getMean();
        if(m[2]<z_filter_min) continue;

        perception_oru::NDTCell *cell;
        pcl::PointXYZ p;
        p.x = m[0];p.y=m[1];p.z=m[2];

        if(map_->getCellAtPoint(p,cell)){
          //if(map.getCellForPoint(p,cell)){
          if(cell == NULL) continue;
          if(cell->hasGaussian_){
            Eigen::Matrix3d covCombined = cell->getCov() + T.rotation()*ndts[n]->getCov() *T.rotation().transpose();
            Eigen::Matrix3d icov;
            bool exists;
            double det = 0;
            covCombined.computeInverseAndDetWithCheck(icov,det,exists);
            if(!exists) continue;
            double l = (cell->getMean() - m).dot(icov*(cell->getMean() - m));
            if(l*0 != 0) continue;
            score += score_cell_weight + (1.0 - score_cell_weight) * exp(-0.05*l/2.0);
          }else{
          }
        }
      }
      pf.pcloud[i].SetLikelihood(score);
    }
  }///#pragma

  t_pseudo = getDoubleTime() - t_pseudo;
  for(unsigned int j=0;j<ndts.size();j++){
    delete ndts[j];
  }
  pf.normalize();
}

void MCLNDTType::SIResampling(){
  if(forceSIR){
    pf.SIRUpdate();
  }
  else{
    double varP=0;
    for(int i = 0; i<pf.size();i++){
      varP += (pf.pcloud[i].GetProbability() - 1.0/pf.size())*(pf.pcloud[i].GetProbability() - 1.0/pf.size());
    }
    varP /= pf.size();
    varP = sqrt(varP);
    //fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf) itr since SIR= %d\n",varP,pf.size(), Nn, t_pred,t_pseudo,sinceSIR_);
    if(varP > /*0.006*/SIR_varP_threshold || sinceSIR_ > /*25*/SIR_max_iters_wo_resampling_){
      //fprintf(stderr,"-SIR- ");
      sinceSIR_ = 0;
      pf.SIRUpdate();
    }
    else{
      sinceSIR_++;
    }
  }
}

void MCLNDTType::ComputeMotionCovar(const Eigen::Affine3d &Tmotion, Eigen::Matrix<double,6,1> &motion_cov ){

  /*  Eigen::Matrix<double, 6,6> motion_model_m(motion_model.data());

  Eigen::Vector3d tr = Tmotion.translation();
  Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
  motion_model_.getCovMatrix(Tmotion);
  ndt_generic::normalizeEulerAngles(rot);
  Eigen::Matrix<double,6,1> incr;
  incr<< fabs(tr[0]),fabs(tr[1]),fabs(tr[2]), fabs(rot[0]), fabs(rot[1]), fabs(rot[2]);
  motion_cov = motion_model_m*incr;*/
  motion_cov = motion_model_.GetCovarianceDiagonal(Tmotion);
  motion_cov+=motion_model_.params.offset;
}

void MCLNDTType::OdometryPrediction(const Eigen::Affine3d &Tmotion, bool disable_noise){

  Eigen::Matrix<double,6,1> motion_cov;
  if(!disable_noise)
    ComputeMotionCovar(Tmotion,motion_cov);
  else
    motion_cov<<0, 0, 0, 0, 0, 0;

  bool new_map_node = graph_map_->SwitchToClosestMapNode(graph_map_->GetCurrentNodePose()*pf.getMean()); //Get mean estimate in world frame and find closest node
  if(new_map_node){
    Eigen::Affine3d Tprev_to_new;
    Tprev_to_new=(graph_map_->GetPreviousNodePose().inverse()*graph_map_->GetCurrentNodePose()).inverse();
    cout<<"change frame, transf="<<Tprev_to_new.translation().transpose()<<endl;
    pf.predict(Tmotion,motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3], motion_cov[4], motion_cov[5],Tprev_to_new);
    map_ = this->GetCurrentNodeNDTMap();
  }
  else
    pf.predict(Tmotion,motion_cov[0], motion_cov[1], motion_cov[2], motion_cov[3], motion_cov[4], motion_cov[5]);
}

bool MCLNDTType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion){

  bool updated=false;
  counter++;


  Eigen::Affine3d Tlast=graph_map_->GetCurrentNodePose()*pf.getMean();
  bool disable_prediciton_noise=false; // if there os
  if( (Tlast.translation()-pose_last_update_.translation()).norm()<0.01 && (pose_last_update_.inverse()*Tlast).rotation().eulerAngles(0,1,2).norm()<0.005 )
    disable_prediciton_noise=true;

  OdometryPrediction(Tmotion,disable_prediciton_noise);
  Eigen::Affine3d Tinit=graph_map_->GetCurrentNodePose()*pf.getMean();

  if(!enable_localisation_ ||disable_prediciton_noise ){
     SetPose(Tinit); //Use only odometry to predict movement
  return updated;
  }


        perception_oru::NDTMap local_map(new perception_oru::LazyGrid(resolution_sensor));
  local_map.loadPointCloud(cloud);
  local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

  std::vector<perception_oru::NDTCell*> ndts_original =local_map.getAllCells();
  std::vector<perception_oru::NDTCell*> ndts_subsampled;

  Subsample(ndts_original,ndts_subsampled);
  AssignParticleScore(ndts_subsampled);
  SIResampling();

  Tinit=graph_map_->GetCurrentNodePose()*pf.getMean();
  pose_last_update_=Tinit;
  SetPose(Tinit);
  if(visualize_){
    geometry_msgs::PoseArray particles_msg= ParticlesToMsg( pf.pcloud);
    part_pub.publish(particles_msg);
  }
  updated=true;
  return updated;
}

bool MCLNDTType::UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion){

  // Filter out the flat parts...
  pcl::PointCloud<pcl::PointXYZ> cornerPointsSharp;
  pcl::PointCloud<pcl::PointXYZ> cornerPointsLessSharp;
  pcl::PointCloud<pcl::PointXYZ> surfPointsFlat;
  pcl::PointCloud<pcl::PointXYZ> surfPointsLessFlat;

  //segmentPointCurvature(cloud, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat);

  //UpdateAndPredict(surfPointsLessFlat, Tmotion);

  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(cloud, cloud_xyz);
  return UpdateAndPredict(cloud_xyz, Tmotion);
}

std::string MCLNDTType::ToString(){

  std::stringstream ss;
  ss<<LocalizationType::ToString();
  ss<<graph_map_->ToString();
  ss<<"NDTMCL:"<<endl;
  ss<<"number of particles: "<<n_particles_<<endl;
  ss<<"resolution: "<<resolution<<endl;
  ss<<"force SIR: "<<std::boolalpha<<forceSIR<<endl;
  ss<<"SIR var Probability threshold: "<<SIR_varP_threshold<<endl;
  ss<<"motion model: ";
  ss<<motion_model_.params.getDescString()<<endl;
  //for (int i = 0; i < motion_model_.params.motion_model.size(); i++) { if (i % 6 == 0) {ss << endl;} ss<<motion_model_.params.motion_model[i] << " " << endl; }
  //ss<<"motion model offset: ";
  //for (int i = 0; i < motion_model_offset.size(); i++) { if (i % 6 == 0) {ss << endl;}; ss<<motion_model_offset[i] << " " << endl; }
  return ss.str();
}

void MCLNDTType::InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance){ //Vector3d variance={Vx, Vy, Vz Vp Vr Vy} pose specified in global frame
  SetPose(pose);
  pose_last_update_=GetPose();
  bool new_map_node = graph_map_->SwitchToClosestMapNode(pose);
  map_ = this->GetCurrentNodeNDTMap();
  Eigen::Affine3d pose_local=graph_map_->GetCurrentNodePose().inverse()*pose;
  Eigen::Vector3d pos=pose_local.translation();
  Eigen::Vector3d euler = pose_local.rotation().eulerAngles(0,1,2);
  normalizeEulerAngles(euler);

  pf.initializeNormalRandom(n_particles_, pos(0),pos(1),pos(2),euler(0),euler(1), euler(2), variance(0),variance(1),variance(2),variance(3),variance(4),variance(5));
  initialized_=true;
}

double MCLNDTType::getDoubleTime()
{
  struct timeval time;
  gettimeofday(&time,NULL);
  return time.tv_sec + time.tv_usec * 1e-6;
}

void MCLNDTType::normalizeEulerAngles(Eigen::Vector3d &euler) {
  euler[0] = angles::normalize_angle(euler[0]);
  euler[1] = angles::normalize_angle(euler[1]);
  euler[2] = angles::normalize_angle(euler[2]);

  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] = -euler[1] + M_PI;
    euler[2] += M_PI;

    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}

geometry_msgs::PoseArray MCLNDTType::ParticlesToMsg(std::vector<particle> &particles){
  geometry_msgs::PoseArray ret;
  Eigen::Affine3d Tmap=graph_map_->GetCurrentNodePose();
  for(int i = 0; i < particles.size(); i++){
    geometry_msgs::Pose particle_msg;
    double x, y, z, r, p, t;
    particles[i].GetXYZ(x, y, z);
    particles[i].GetRPY(r, p, t);
    Eigen::Affine3d pose_particle_wf=Tmap*ndt_generic::xyzrpyToAffine3d(x,y,z,r,p,t);
    tf::poseEigenToMsg(pose_particle_wf,particle_msg);
    ret.poses.push_back(particle_msg);

  }
  ret.header.stamp=ros::Time::now();
  ret.header.frame_id = "world";
  return ret;
}
MCLNDTParam::MCLNDTParam(){

  /*motion_model.push_back(0.05);
  motion_model.push_back(0.05);
  motion_model.push_back(0.02);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.02);

  motion_model.push_back(0.05);
  motion_model.push_back(0.1);
  motion_model.push_back(0.02);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.02);


  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.001);
  motion_model.push_back(0.001);
  motion_model.push_back(0.001);

  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);

  motion_model.push_back(0.01);
  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.01);

  motion_model.push_back(0.1);
  motion_model.push_back(0.01);
  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);

  /*motion_model_offset.push_back(0.00);
  motion_model_offset.push_back(0.002);
  motion_model_offset.push_back(0.0000001);//0.002
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.001);

  motion_model_offset.push_back(0.005);
  motion_model_offset.push_back(0.005);
  motion_model_offset.push_back(0.0000001);//0.002
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.003);*/
}

std::string MCLNDTParam::ToString(){
  stringstream ss;
  ss<<LocalisationParam::ToString();
  ss<<"MCL-NDT parameters:"<<endl;
  ss<<"z_filter_min="<<z_filter_min<<endl;
  ss<<"n_particles="<<n_particles<<endl;
  ss<<"SIR_max_iters_wo_resampling="<<SIR_max_iters_wo_resampling<<endl;
  ss<<"forceSIR="<<std::boolalpha<<forceSIR<<endl;
  return ss.str();
}

void  MCLNDTParam::GetParamFromRos(){
  LocalisationParam::GetParamFromRos();
  cout<<"MCLNDT localisation parameters from ROS"<<endl;
  ros::NodeHandle nh("~");//base class parameters
  nh.param("z_filter_height",z_filter_min,-10000.0);
  nh.param("particle_count",n_particles,250);
  nh.param("SIR_max_iters_wo_resampling",SIR_max_iters_wo_resampling,30);
  nh.param("force_SIR",forceSIR,false);
  cout<<"Fetched localisation parameters from ros"<<endl;
  //cout<<ToString()<<endl;

}


}
}
