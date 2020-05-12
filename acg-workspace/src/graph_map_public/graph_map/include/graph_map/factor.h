#ifndef GRAPHFACTOR_H
#define GRAPHFACTOR_H
#include "graphfactory.h"
#include "graph_map/map_node.h"
#include "graph_map/map_type.h"
#include "Eigen/Dense"
#include "boost/serialization/serialization.hpp"
#include "ndt_generic/serialization.h"
namespace perception_oru{
namespace graph_map{
typedef enum factoryype{observationFactor=0,poseFactor=1}factorType;
class factor{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//<<<<<<< HEAD
//
//  virtual bool Connects(NodePtr node);
//  virtual void GetNodes(NodePtr& prev, NodePtr& next){prev=prev_;next=next_;}
//=======

  virtual NodePtr GetPrev(){return prev_;}

  virtual NodePtr GetNext(){return next_;}

  virtual bool Connects(NodePtr node);

  virtual void GetNodes(NodePtr& prev, NodePtr& next){prev=prev_;next=next_;}
//>>>>>>> port-kinetic
  //factor();
  //factor(mapNodePtr prev, mapNodePtr next){prev_=prev; next_=next;}
  factor(MapNodePtr prev, NodePtr next,const Eigen::Affine3d &diff,const Matrix6d &cov){

    prev_=prev;
    next_=next;
    diff_=diff;
    covar_=cov;
  }
  void UpdateFactor(const Eigen::Affine3d &diff){diff_=diff;}
  factor();
  virtual Eigen::Affine3d GetDiff() const{return diff_;}
  virtual Matrix6d GetCovariance() const{return covar_;}
  
protected:
  //factorType factortype_;
  Eigen::Affine3d diff_;
  NodePtr prev_,next_;
  Matrix6d covar_;
private:
  friend class GraphFactory;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
  //ar & factortype_;
    ar & diff_;
    ar & prev_;
    ar & next_;
    ar & covar_;
  }


};
}
}

#endif // FACTOR_H
