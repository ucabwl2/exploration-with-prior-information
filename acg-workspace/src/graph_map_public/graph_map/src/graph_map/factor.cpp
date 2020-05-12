#include "graph_map/factor.h"
namespace perception_oru{
namespace graph_map{
bool factor::Connects(NodePtr node){
  if(node->GetId()==prev_->GetId()|| node->GetId()==next_->GetId()){
    return true;
  }
  else return false;
}
factor::factor(){
  diff_=Eigen::Affine3d::Identity();
  prev_=NULL;
  next_=NULL;
  covar_=unit_covar;
}

}

}
