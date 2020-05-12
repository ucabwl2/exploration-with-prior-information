#ifndef NDT_CONVERSIONS_HH
#define NDT_CONVERSIONS_HH

#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/NDTCellMsg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>


namespace perception_oru{
inline bool toMessage(const std::vector<perception_oru::NDTCell*> &map_vector, ndt_map::NDTMapMsg &msg, const std::string &frame_name){

  msg.header.stamp=ros::Time::now();
  msg.header.frame_id=frame_name;//is it in *map?



  for (int cell_idx=0;cell_idx<map_vector.size();cell_idx++){
  if(map_vector[cell_idx] != NULL){ //we send intialized cells

    ndt_map::NDTCellMsg cell;
    cell.hasGaussian_ = map_vector[cell_idx]->hasGaussian_;

    cell.center_x = map_vector[cell_idx]->getCenter().x;
    cell.center_y = map_vector[cell_idx]->getCenter().y;
    cell.center_z = map_vector[cell_idx]->getCenter().z;

    if(map_vector[cell_idx]->hasGaussian_){ //we only send a cell with gaussian

      Eigen::Vector3d means=map_vector[cell_idx]->getMean();
      cell.mean_x=means(0);
      cell.mean_y=means(1);
      cell.mean_z=means(2);
      cell.occupancy=map_vector[cell_idx]->getOccupancyRescaled();
      Eigen::Matrix3d cov=map_vector[cell_idx]->getCov();
      for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        cell.cov_matrix.push_back(cov(i,j));
      }
      }
      cell.N=map_vector[cell_idx]->getN();
    }
    msg.cells.push_back(cell);
  }
  }
  return true;
}
  /** 
   *
   * \brief Message building fucntion
   * \details Converts an object of type NDTMap into NDTMapMsg
   * message. Message contain all the data strored in the object. 
   * @param[in] map Pointer to NDTMap object.
   * @param[out] msg formated message
   * @param[in] frame_name name of the coordination frame for the transformed map
   *   
   */
  inline bool toMessage(NDTMap *map, ndt_map::NDTMapMsg &msg,std::string frame_name){
    std::vector<perception_oru::NDTCell*> map_vector=map->getAllInitializedCells();
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id=frame_name;//is it in *map?
    if(!map->getGridSizeInMeters(msg.x_size,msg.y_size,msg.z_size)){
	    ROS_ERROR("NO GRID SIZE");
	    return false;
    }
    if(!map->getCentroid(msg.x_cen,msg.y_cen,msg.z_cen)){
	    ROS_ERROR("NO GRID CENTER");
	    return false;
    }
    if(!map->getCellSizeInMeters(msg.x_cell_size,msg.y_cell_size,msg.z_cell_size)){
	    ROS_ERROR("NO CELL SIZE");
	    return false;
    }
    for (int cell_idx=0;cell_idx<map_vector.size();cell_idx++){
		if(map_vector[cell_idx] != NULL){ //we send intialized cells
			
			ndt_map::NDTCellMsg cell;
			cell.hasGaussian_ = map_vector[cell_idx]->hasGaussian_;
			
			cell.center_x = map_vector[cell_idx]->getCenter().x;
			cell.center_y = map_vector[cell_idx]->getCenter().y;
			cell.center_z = map_vector[cell_idx]->getCenter().z;

			cell.occupancy=map_vector[cell_idx]->getOccupancyRescaled();
			cell.occupancy_raw=map_vector[cell_idx]->getOccupancy();

			if(map_vector[cell_idx]->hasGaussian_){ //we only send a cell with gaussian
			
				Eigen::Vector3d means=map_vector[cell_idx]->getMean();
				cell.mean_x=means(0);
				cell.mean_y=means(1);
				cell.mean_z=means(2);
				Eigen::Matrix3d cov=map_vector[cell_idx]->getCov();
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						cell.cov_matrix.push_back(cov(i,j));
					}
				}
				cell.N=map_vector[cell_idx]->getN();
			}
			msg.cells.push_back(cell);
		}
//		delete map_vector[cell_idx];
    }
    return true;
  }
  /** 
   *
   * \brief from message to NDTMap object
   * \details Converts ndt map message into a NDTMap object
   * @param[in,out] idx Pointer to lazy grid of the new NDTMap
   * @param[out] map Pointer to NDTMap object
   * @param[in] msg message to be converted
   * @param[out] frame_name name of the coordination frame of the map
   * @param[in] dealloc if set to true, the NDTMap with deallocate the memory of LazyGrid in its destructor. Default set to false.
   * @param[in] get_cell_with_no_gaussian if set to true, cell with no gaussian but initialized are added to the map. Usefull to keep NDT-OM
   *   
   */
  inline bool fromMessage(LazyGrid* &idx, NDTMap* &map, ndt_map::NDTMapMsg msg, std::string &frame_name, bool dealloc = false, bool get_cell_with_no_gaussian = true){
    if(!(msg.x_cell_size==msg.y_cell_size&&msg.y_cell_size==msg.z_cell_size)){ //we assume that voxels are cubes
	  ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE"); 
	  return false;
    }
    idx=new LazyGrid(msg.x_cell_size);
    map = new NDTMap(idx,msg.x_cen,msg.y_cen,msg.z_cen,msg.x_size,msg.y_size,msg.z_size, dealloc);
    frame_name=msg.header.frame_id;
    int gaussians=0;
    for(int itr=0;itr<msg.cells.size();itr++){
		if(msg.cells[itr].hasGaussian_ == true){
			Eigen::Vector3d mean;
			Eigen::Matrix3d cov;
			mean<<msg.cells[itr].mean_x,msg.cells[itr].mean_y,msg.cells[itr].mean_z;
			int m_itr=0;
			for(int i=0;i<3;i++){
				for(int j=0;j<3;j++){
				cov(i,j)=msg.cells[itr].cov_matrix[m_itr];
				m_itr++;
				}
			}
			map->addDistributionToCell(cov,mean,msg.cells[itr].N);
//			std::cout << "Adding initialized cell with gaussian at " << mean(0) << " " << mean(1) << " " << mean(2) << std::endl;
		}
		else{
			if(get_cell_with_no_gaussian == true){
//				std::cout << "Adding initialized cell with no gaussian" << std::endl;
				perception_oru::NDTCell* ptCell;
				pcl::PointXYZ point;
	// 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
				point.x = msg.cells[itr].center_x;
	// 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
				point.y = msg.cells[itr].center_y;
	// 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
				point.z = msg.cells[itr].center_z;
	// 			std::cout << "Getting the Cell" << std::endl;
				map->getCellAtAllocate(point, ptCell);
	// 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
//				ptCell->updateOccupancy(-0.2);
	// 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
//				if(ptCell->getOccupancy()<=0)
				ptCell->hasGaussian_ = false;
				ptCell->setOccupancy(msg.cells[itr].occupancy_raw);
//				std::cout << "Occupancy " << ptCell->getOccupancy() << std::endl;

				int occ_rescaled = ptCell->getOccupancyRescaled() * 100;
				int occ_map = msg.cells[itr].occupancy * 100;

				assert(occ_rescaled == occ_map);
	// 			std::cout << "Adding initialized cell with no gaussian" << std::endl;
			}
		}
    }
    return true;
  }
  /**
   *
   * \brief builds ocuupancy grid message
   * \details Builds 2D occupancy grid map based on 2D NDTMap
   * @param[in] ndt_map 2D ndt map to conversion
   * @param[out] occ_grid 2D cost map
   * @param[in] resolution desired resolution of occupancy map
   * @param[in] name of cooridnation frame for the map (same as the NDT map has)
   * 
   */
  inline bool toOccupancyGrid(NDTMap *ndt_map, nav_msgs::OccupancyGrid &occ_grid, double resolution,std::string frame_id, double scaling_factor_gaussian = 0.05, bool use_euclidean_distance = false){//works only for 2D case
    double size_x, size_y, size_z;
    int size_x_cell_count, size_y_cell_count;
    double cen_x, cen_y, cen_z;
    double orig_x, orig_y;
    ndt_map->getGridSizeInMeters(size_x,size_y,size_z);
    ndt_map->getCentroid(cen_x,cen_y,cen_z);
    orig_x=cen_x-size_x/2.0;
    orig_y=cen_y-size_y/2.0;
    size_x_cell_count=int(size_x/resolution);
    size_y_cell_count=int(size_y/resolution);
    occ_grid.info.width=size_x_cell_count;
    occ_grid.info.height=size_y_cell_count;
    occ_grid.info.resolution=resolution;
    occ_grid.info.map_load_time=ros::Time::now();
    occ_grid.info.origin.position.x=orig_x;
    occ_grid.info.origin.position.y=orig_y;
    occ_grid.header.stamp=ros::Time::now();
    occ_grid.header.frame_id=frame_id;
    // for(double py=orig_y+resolution/2.0;py<orig_y+size_x;py+=resolution){
    //   for(double px=orig_x+resolution/2.0;px<orig_x+size_x;px+=resolution){
    for(int iy = 0; iy < size_y_cell_count; iy++) {
      for(int ix = 0; ix < size_x_cell_count; ix++) {
        double px = orig_x + resolution*ix + resolution*0.5;
        double py = orig_y + resolution*iy + resolution*0.5;

        pcl::PointXYZ pt(px,py,0);
        perception_oru::NDTCell *cell;
        if(!ndt_map->getCellAtPoint(pt, cell)){
          occ_grid.data.push_back(-1);
        }
        else if(cell == NULL){
          occ_grid.data.push_back(-1);
        }
        else{
	        /* TODO Will be problematic around wall:
	         * It might put the cell to -1 (as in occupied are unknown since it's not close enough
	         * to the wall to be occupied
	         * However, if the cells around are seen empty, then that cell should be empty also.
	         * TO FIX
	         */
//          if(cell->hasGaussian_) {
	          Eigen::Vector3d vec(pt.x, pt.y, pt.z);
	          vec = vec - cell->getMean();

	        double likelihood = 0;
	        if(use_euclidean_distance){
		        likelihood = vec.norm();
	        }
	        else{
		        likelihood = vec.dot(cell->getInverseCov() * vec);
	        }
//	          double likelihood = vec.dot(cell->getInverseCov() * vec);
//	          double likelihood = vec.norm();
	          char s_likelihood;
	          if (cell->getOccupancy() != 0.0) {
		          if (cell->getOccupancy() > 0.0) {
			          if (std::isnan(likelihood)) s_likelihood = -1;
//			          std::cout << "SCALIN GAUSSIAN OCC " << scaling_factor_gaussian << std::endl;

			          //HACK To remove
//			          Eigen::Vector3d& eigenval;
//			          Eigen::Matrix3d& eigenvec
//			          eigenval = cell.getEvals();
//			          eigenvec = cell.getEvecs();
//			          EigenSort2D(eigenval, eigenvec);



			          likelihood = exp(-likelihood * scaling_factor_gaussian);
//			          likelihood = (0.1 + 0.9 * likelihood);
			          s_likelihood = char(likelihood * 100.0);
			          if (likelihood > 1.0) s_likelihood = 100;
			          occ_grid.data.push_back(s_likelihood);
		          } else {
//			          std::cout << "Empty cell in occupancy !" << std::endl;
			          occ_grid.data.push_back(0);
		          }
	          } else {
		          occ_grid.data.push_back(-1);
	          }
//          }
//          else{
//	          std::cout << "ADDING CELL WITH NO GAUSSIAN " << std::endl;
//	          if (cell->getOccupancy() != 0.0) {
//		          if (cell->getOccupancy() > 0.0) {
//			          double occ_val = cell->getOccupancyRescaled() * 100;
//			          std::cout << "With value " << occ_val << std::endl;
//			          occ_grid.data.push_back(occ_val);
//		          } else {
//			          std::cout << "Empty CELL :D ! " << std::endl;
//			          occ_grid.data.push_back(0);
//		          }
//	          } else {
//		          occ_grid.data.push_back(-1);
//	          }
//
//          }
        }
      }
    }    
    return true;
  }

/**
   *
   * \brief builds ocuupancy grid message
   * \details Builds 2D occupancy grid map based on 2D NDTMap
   * @param[in] ndt_map 2D ndt map to conversion
   * @param[out] occ_grid 2D cost map
   * @param[in] resolution desired resolution of occupancy map
   * @param[in] name of cooridnation frame for the map (same as the NDT map has)
   * 
   */
  inline bool toOccupancyGrid(const boost::shared_ptr<perception_oru::NDTMap>& ndt_map, nav_msgs::OccupancyGrid &occ_grid, double resolution,std::string frame_id){//works only for 2D case
    double size_x, size_y, size_z;
    int size_x_cell_count, size_y_cell_count;
    double cen_x, cen_y, cen_z;
    double orig_x, orig_y;
    ndt_map->getGridSizeInMeters(size_x,size_y,size_z);
    ndt_map->getCentroid(cen_x,cen_y,cen_z);
    orig_x=cen_x-size_x/2.0;
    orig_y=cen_y-size_y/2.0;
    size_x_cell_count=int(size_x/resolution);
    size_y_cell_count=int(size_y/resolution);
    occ_grid.info.width=size_x_cell_count;
    occ_grid.info.height=size_y_cell_count;
    occ_grid.info.resolution=resolution;
    occ_grid.info.map_load_time=ros::Time::now();
    occ_grid.info.origin.position.x=orig_x;
    occ_grid.info.origin.position.y=orig_y;
    occ_grid.header.stamp=ros::Time::now();
    occ_grid.header.frame_id=frame_id;
    // for(double py=orig_y+resolution/2.0;py<orig_y+size_x;py+=resolution){
    //   for(double px=orig_x+resolution/2.0;px<orig_x+size_x;px+=resolution){
    for(int iy = 0; iy < size_y_cell_count; iy++) {
      for(int ix = 0; ix < size_x_cell_count; ix++) {
        double px = orig_x + resolution*ix + resolution*0.5;
        double py = orig_y + resolution*iy + resolution*0.5;

        pcl::PointXYZ pt(px,py,0);
        perception_oru::NDTCell *cell;
        if(!ndt_map->getCellAtPoint(pt, cell)){
          occ_grid.data.push_back(-1);
        }
        else if(cell == NULL){
          occ_grid.data.push_back(-1);
        }
        else{
          Eigen::Vector3d vec (pt.x,pt.y,pt.z);
          vec = vec-cell->getMean();                  
          double likelihood = vec.dot(cell-> getInverseCov()*vec);
          char s_likelihood;
          if(cell->getOccupancy()!=0.0){
            if(cell->getOccupancy()>0.0){
            if(std::isnan(likelihood)) s_likelihood = -1;
            likelihood = exp(-likelihood/2.0) + 0.1;
            likelihood = (0.5+0.5*likelihood);
            s_likelihood=char(likelihood*100.0);
            if(likelihood >1.0) s_likelihood =100;
            occ_grid.data.push_back(s_likelihood);
            }
            else{
              occ_grid.data.push_back(0);
            }
          }
          else{
             occ_grid.data.push_back(-1);
          }
        }
      }
    }    
    return true;
  } 
}
#endif
