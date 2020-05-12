#include "graph_map/ndt/ndt_map_type.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(perception_oru::graph_map::NDTMapType)
namespace perception_oru{
  namespace graph_map{
  using namespace std;

  using namespace perception_oru;

  NDTMapType::NDTMapType( MapParamPtr paramptr) : MapType(paramptr){
    NDTMapParamPtr param = boost::dynamic_pointer_cast< NDTMapParam >(paramptr);//Should not be NULL
    if(param!=NULL){
//      std::cout << "Creating " << std::endl;
      resolution_=param ->resolution_;
//      std::cout << "Creating " << std::endl;
      map_ = new perception_oru::NDTMap(new perception_oru::LazyGrid(resolution_));
//      std::cout << "Creating " << std::endl;
      map_->initialize(0.0,0.0,0.0,param->sizex_,param->sizey_,param->sizez_);
//      std::cout << "Creating " << std::endl;
    }
    else
      cerr<<"Cannot create instance of NDTmapHMT"<<std::endl;
  }
  NDTMapType::~NDTMapType(){}

  void NDTMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

    if(initialized_ && cloud.size()>0){
      ROS_DEBUG_STREAM("Init");
      Eigen::Vector3d localMapSize(2*max_range_,2*max_range_,sizez_);
      if (!simple) {
        map_->addPointCloudMeanUpdate(Tsensor.translation(),cloud,localMapSize, 1e5, /*25*/255, sizez_, 0.06);
        ROS_DEBUG_STREAM("not simple" );
      }
      else {
        ROS_DEBUG_STREAM("Simple");
        map_->addPointCloudSimple(cloud, sizez_);
        map_->computeNDTCells();
      }
    }
    else if(!initialized_ && cloud.size()>0 ){
      ROS_DEBUG_STREAM("Not Init");
      InitializeMap(Tsensor,cloud, simple);
      initialized_ = true;
    }

    ROS_DEBUG_STREAM("Map cells end " << map_->getAllCells().size() );

  }

  void NDTMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

    cerr << "TODO: implement update for point type PointXYZIR - will convert to PointXYZ for now" << endl;
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::copyPointCloud(cloud, cloud_xyz);
    update(Tsensor, cloud_xyz, simple);
  }

  void NDTMapType::InitializeMap(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple){
    ROS_DEBUG_STREAM("initialize map");
    if (!simple) {
      ROS_DEBUG_STREAM( "Not simple" );

      ROS_DEBUG_STREAM( "Map cells before " << map_->getAllCells().size() << "point could size " << cloud.points.size() );
      map_->addPointCloud(Tsensor.translation(),cloud, 0.1, 100.0, 0.1);
      map_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(), 0.1);

      ROS_DEBUG_STREAM("Map cells end of before " << map_->getAllCells().size() );
    }
    else {
      ROS_DEBUG_STREAM( "Simple" );
      map_->addPointCloudSimple(cloud, sizez_);
      map_->computeNDTCells();
    }
  }
  void NDTMapType::GetCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, bool ndt_filtered_cloud_){
    if(ndt_filtered_cloud_){
      NDTCell *cell=new NDTCell();
      for(int i=0;i<input_cloud_.size();i++){
        for(int j=0;j<input_cloud_[i].size();j++){
          if(map_->getCellAtPoint(input_cloud_[i][j],cell) && cell->hasGaussian_){
            cloud.push_back(input_cloud_[i][j]);
          }
        }
      }
    }
    else if(ndt_cell_cloud_){
      NDTMap *ndt_map= GetNDTMap();
      std::vector <NDTCell*> cells=ndt_map->getAllCells();
      for(int i=0;i<cells.size();i++){
        Eigen::Vector3d pt=cells[i]->getMean();
        pcl::PointXYZ pcl_point(pt(0),pt(1),pt(2));
        cloud.push_back(pcl_point);
      }
    }
    else
      MapType::GetCloud(cloud,true);
  }
  bool NDTMapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){

    Affine3d Tdiff=Affine3d::Identity();
    Tdiff=T_source.inverse()*T_target;
    pcl::PointXYZ center_pcl(Tdiff.translation()(0),Tdiff.translation()(1),Tdiff.translation()(2));
    if( NDTMapPtr targetPtr=boost::dynamic_pointer_cast<NDTMapType>(target) ){
      ROS_DEBUG_STREAM("dynamic casted pointer");
      if(resolution_!=targetPtr->resolution_)//checking if source and target have same resolution, they shoould have.
        return false;

      if(radius==-1)//if radius is not defined, match rcenter_pcladius to size of new map
        radius=targetPtr->sizex_<targetPtr->sizey_? targetPtr->sizex_/2:targetPtr->sizey_/2;

      int neighboors=radius/resolution_;
      ROS_DEBUG_STREAM("neighboors cells to search through="<<neighboors);
      std::vector<NDTCell*>cells= map_->getCellsForPoint(center_pcl,neighboors,true);
      ROS_DEBUG_STREAM("cells to transfer:"<<cells.size());
      Tdiff=T_source.inverse()*T_target;
      ROS_DEBUG_STREAM("centerpoint in prev map frame=\n"<<Tdiff.translation());

      for(int i=0;i<cells.size();i++){
        Eigen::Matrix3d cov=Tdiff.inverse().linear()*cells[i]->getCov()*Tdiff.linear();
        Eigen::Vector3d mean=Tdiff.inverse()*cells[i]->getMean();
        targetPtr->GetNDTMap()->addDistributionToCell(cov,mean,3);
      }

    }

  }
  std::string NDTMapType::ToString(){
    stringstream ss;
    ss<<MapType::ToString()<<"NDT Map Type:"<<endl;
    ss<<"resolution:"<<resolution_<<endl;
    ss<<"resolution local factor:"<<resolution_local_factor_<<endl;
    ss<<"nr active cells:"<<map_->numberOfActiveCells() << endl;
    return ss.str();
  }

}
}
