/**
@file    wb_traj_in_out.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Dec 13, 2016
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_WB_TRAJ_IN_OUT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_WB_TRAJ_IN_OUT_H_

#include <xpp/utils/state.h>
#include <Eigen/Dense>
#include <array>

namespace xpp {
namespace opt {

template<size_t N_EE>
class Node  {
public:
  using BaseLin3d    = xpp::utils::StateLin3d;
  using BaseAng3d    = xpp::utils::StateAng3d;
  using BaseLin1d    = xpp::utils::StateLin1d;
  using Vector3d     = Eigen::Vector3d;
  using FeetArray    = std::array<BaseLin3d, N_EE>;
  using ContactArray = std::array<bool, N_EE>;

  static const int kNee = N_EE;

  FeetArray feet_W_;
  ContactArray swingleg_;
  BaseAng3d base_ang_;
  BaseLin1d base_z_;
  double T; ///< time to reach this state

  double GetZAvg() const
  {
    double z_avg = 0.0;
    for (auto f : feet_W_) {
      z_avg += (f.p.z()/kNee);
    }

    return z_avg;
  }
};


template<size_t N_EE>
class ArticulatedRobotState {
public:
  using BaseState     = xpp::utils::State3d;
  using SplineNode    = Node<N_EE>;
  using FeetArray     = typename SplineNode::FeetArray;
  using ContactArray  = typename SplineNode::ContactArray;

  BaseState base_;
  FeetArray feet_W_;
  ContactArray swingleg_;
  double t_;
};

} // namespace opt
} // namespace xpp


#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_WB_TRAJ_IN_OUT_H_ */
