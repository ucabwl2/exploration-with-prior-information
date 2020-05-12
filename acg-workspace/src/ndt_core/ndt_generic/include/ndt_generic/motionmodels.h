#ifndef NDT_GENERIC_MOTIONMODELS_H
#define NDT_GENERIC_MOTIONMODELS_H
#include "ndt_generic/motion_model_3d.h"
//Until ported this to YAML file
namespace perception_oru
{
#if 0
inline bool GetMotionModel(const std::string &dataset, std::vector<double> &motion_model, std::vector<double> &motion_model_offset){

 if(dataset.compare("hx")==0 || dataset.compare("volvo_2017_12_01")==0){
   std::cout<<"hx motion settings applied"<<std::endl;

   motion_model.clear();

   motion_model.push_back(0.01);
   motion_model.push_back(0.002);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.005);
   motion_model.push_back(0.001);

   motion_model.push_back(0.002);
   motion_model.push_back(0.005);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.005);

   motion_model.push_back(0.005);
   motion_model.push_back(0.001);
   motion_model.push_back(0.01);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.005);

   motion_model.push_back(0.002);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.01);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);

   motion_model.push_back(0.005);
   motion_model.push_back(0.002);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.04);
   motion_model.push_back(0.001);

   motion_model.push_back(0.005);
   motion_model.push_back(0.002);
   motion_model.push_back(0.0001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.001);
   motion_model.push_back(0.01);

   motion_model_offset.push_back(0.01);
   motion_model_offset.push_back(0.01);
   motion_model_offset.push_back(0.01);
   motion_model_offset.push_back(0.001);
   motion_model_offset.push_back(0.002);
   motion_model_offset.push_back(0.001);
 }
 else if(dataset.compare("arla-2012")==0){
   std::cout<<"arla-2012 motion settings applied"<<std::endl;
     motion_model.clear();
     motion_model.push_back(0.05);
     motion_model.push_back(0.05);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.04);

     motion_model.push_back(0.05);
     motion_model.push_back(0.1);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.04);


     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.01);
     motion_model.push_back(0.01);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.02);

     motion_model_offset.push_back(0.0005);
     motion_model_offset.push_back(0.0005);
     motion_model_offset.push_back(0.000000000);//0.002
     motion_model_offset.push_back(0.0000000);//0.001
     motion_model_offset.push_back(0.0000000);//0.001
   motion_model_offset.push_back(0.0003);

 }

}
#endif

bool GetMotionModel(const std::string &dataset,MotionModel3d &motionModel){
  MotionModel3d::Params par;
  if(dataset=="arla-2012"){
    par.set2DParams(0.5, 0.5, 1.0, 2, 0.3, 2.5);
    Eigen::Matrix<double,6,1> offset;
    offset<<0.01, 0.01, 0.000, 0.000, 0.000, 0.02;
    par.SetOffset(offset);
  }
  if(dataset=="orkla-velodyne"){

    par.set2DParams(0.5, 0.5, 1.0, 2.0, 0.3, 2.5);
    Eigen::Matrix<double,6,1> offset;
    offset<<0.01, 0.01, 0.00, 0.00, 0.00, 0.02;
    par.SetOffset(offset);
  }
  else if(dataset=="michigan"){
    par.set3DParams(0.05,  0.05, //x
                    0.05 , 0.05,//y
                    0.05, 0.05, //z
                    0.05,  0.05, //ex
                    0.05,  0.05, //ey
                    0.2, 0.2); //ez
    Eigen::Matrix<double,6,1> offset;
    offset<<0.003, 0.003, 0.003, 0.001, 0.001, 0.001;
    par.SetOffset(offset);
    std::cout<<"Using motion model \"michigan\""<<std::endl;
  }
  else if(dataset=="volvo_2017_12_01" || dataset.compare("hx")==0 ){
    par.set3DParams
                       (0.08, 0.0, //x
                        0.08, 0.08, //y
                        0.16, 0.0, //z
                        0.03, 0.0, //ex
                        0.05, 0.0, //ey
                        0.08, 0.08); //ez

    Eigen::Matrix<double,6, 1> offset;
    offset<<0.003, 0.003, 0.003, 0.001, 0.001, 0.001;
    par.SetOffset(offset);
  }

  motionModel.setParams(par);

}
}
#endif // MOTIONMODELS_H
