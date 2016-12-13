/**
 @file    hyq_joint_mapper.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 12, 2016
 @brief   Brief description
 */

#include <xpp/hyq/hyq_joint_mapper.h>
#include <xpp/hyq/hyq_endeffectors.h>

namespace xpp {
namespace hyq {

HyqJointMapper::HyqJointMapper ()
{
  // TODO Auto-generated constructor stub
}

HyqJointMapper::~HyqJointMapper ()
{
  // TODO Auto-generated destructor stub
}

HyqJointMapper::SplineNode
HyqJointMapper::BuildSplineNode (const HyqState& state) const
{
  SplineNode node;
//  node.swingleg_ = state.swingleg_;
  node.base_ang_ = state.base_.ang;
  node.base_z_   = state.base_.lin.Get1d(utils::Z);

//  LegDataMap<BaseLin3d> feet;
  auto W_p_ee = state.GetEEInWorld();
  auto W_v_ee = state.GetEEInVelWorld();

  // zmp_ clean this
//  for (int leg=0; leg<W_p_ee.size(); ++leg) {
  for (auto leg : LegIDArray) {
//    opt::EndeffectorID ee = static_cast<opt::EndeffectorID>(leg); // assume there are ordered like this
    opt::EndeffectorID ee = kMapHyqToOpt.at(leg);
    node.feet_W_.at(ee).p = W_p_ee.at(leg);
    node.swingleg_.at(ee) = state.swingleg_.at(leg);
//    node.feet_W_.at(ee).v = W_v_ee[leg];
//    node.feet_W_.at(ee).a.setZero(); // mpc fill this at some point as well
  }

  node.T = 0.0;
  return node;
}

HyqJointMapper::HyqStateVec
HyqJointMapper::BuildWholeBodyTrajectoryJoints (const ArtiRobVec& robot_vec) const
{
  HyqStateVec trajectory_joints;

  HyqState hyq_j_prev;
  double t_prev = 0.0;
  bool first_state = true;
  for (const auto& state : robot_vec) {

    HyqState hyq_j;
    hyq_j.base_     = state.base_;

    // convert generic endeffectors to hyq and extract only positions
    HyqState::PosEE pose_ee_W;
    for (int i=0; i<state.feet_W_.size(); ++i) {
      auto ee  = static_cast<opt::EndeffectorID>(i); // assume there are ordered like this
      auto leg = kMapOptToHyq.at(ee);
      pose_ee_W.at(leg)       = state.feet_W_.at(ee).p;
      hyq_j.swingleg_.at(leg) = state.swingleg_.at(ee);
    }

    // joint position through inverse kinematics
    hyq_j.SetJointAngles(pose_ee_W);

    // joint velocity
    if (!first_state) { // to avoid jump in vel/acc in first state
      double dt = state.t_ - t_prev;
      hyq_j.qd  = (hyq_j.q  - hyq_j_prev.q)  / dt;
      hyq_j.qdd = (hyq_j.qd - hyq_j_prev.qd) / dt;
    }

    trajectory_joints.push_back(hyq_j);
    hyq_j_prev = hyq_j;
    t_prev = state.t_;
    first_state = false;
  }

  return trajectory_joints;
}

} /* namespace hyq */
} /* namespace xpp */
