/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef NDT_MATCHER_D2D_2DSC_HH
#define NDT_MATCHER_D2D_2DSC_HH

#include <ndt_map/ndt_map.h>
#include "pcl/point_cloud.h"
#include "ndt_registration/ndt_matcher_d2d_2d.h"
#include "Eigen/Core"
#include "ndt_generic/eigen_utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

namespace perception_oru
{

/**
 * This class implements NDT registration for 3D point cloud scans.
 */
class NDTMatcherD2DSC_2D : public perception_oru::NDTMatcherD2D_2D
{
public:
    /**
     parametrized constructor. A default set is (false,false,true,empty_vector). parameters are:
    \param _isIrregularGrid --- experimental single pass through an irregular grid. also unstable
    \param useDefaultGridResolutions --- if set, the following parameter is set to a preset value
    \param _resolutions --- if previous bool is not set, these are the resolutions (in reverse order) that we will go through
    */
  NDTMatcherD2DSC_2D() : NDTMatcherD2D_2D()
  {
    nb_match_calls = 0;
    nb_success_reg = 0;
    regularize=true;
  }
  NDTMatcherD2DSC_2D(bool lock_zrp_mot) : NDTMatcherD2D_2D()
  {
    nb_match_calls = 0;
    nb_success_reg = 0;
    regularize=true;
  }
  NDTMatcherD2DSC_2D(bool _isIrregularGrid,
                  bool useDefaultGridResolutions, std::vector<double> _resolutions) : NDTMatcherD2D_2D(_isIrregularGrid,useDefaultGridResolutions,_resolutions)
  {
    nb_match_calls = 0;
    nb_success_reg = 0;
    regularize=true;
  }

  /**
     * Registers a point cloud to an NDT structure.
     * \param  fixed
     *   Reference data.
     * \param  moving
     *   The output transformation registers this point cloud to \c fixed.
     * \param  T
     *   This is an input/output parameter. The initial value of \c T
     *   gives the initial pose estimate of \c moving. When the
     *   algorithm terminates, \c T holds the registration result.
     * \param Tcov
     *   Covariance of the input pose parameter. This will be used to form
     *   an additional cost in the objective.
     */
    bool match( const NDTMap& target,
                const NDTMap& source,
                Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                const Eigen::Matrix3d& Tcov);

    //perform line search to find the best descent rate (Mohre&Thuente)
    double lineSearchMTSC(Eigen::Matrix<double,3,1> &increment,
                          std::vector<NDTCell*> &sourceNDT,
                          const NDTMap &targetNDT,
                          const Eigen::Matrix<double,3,1> &localpose,
                          const Eigen::Matrix3d &Q,
                          const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);



    // compare the difference in scores for different local pose offsets. The assumption is that the T and TCov is expressed in the same frame
    /*void scoreComparision( const NDTMap& targetNDT,
                           const NDTMap& scoureNDT,
                           const Eigen::Affine3d& T ,
                           const Eigen::Matrix3d& Tcov,
                           double &score_NDT,
                           double &score_NDT_SC,
                           const Eigen::Affine3d &offset,
                           const Eigen::Affine3d &odom_offset,
                           double alpha);*/


  bool regularize;
  double a;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace

#endif