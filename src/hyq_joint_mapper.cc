/**
 @file    hyq_joint_mapper.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 12, 2016
 @brief   Brief description
 */

#include <xpp/hyq/hyq_joint_mapper.h>

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

HyqJointMapper::HyqStateVec
HyqJointMapper::BuildWholeBodyTrajectoryJoints (const ArtiRobVec& robot_vec) const
{
  HyqStateVec trajectory_joints;

  HyqState hyq_j_prev;
  bool first_state = true;
  for (const auto& state : robot_vec) {

    HyqState hyq_j;
    hyq_j.base_     = state.base_;
    hyq_j.swingleg_ = state.swingleg_;


    HyqState::PosEE ee_W = {state.feet_W_[LF].p, state.feet_W_[RF].p, state.feet_W_[LH].p, state.feet_W_[RH].p};

    // joint position through inverse kinematics
    hyq_j.SetJointAngles(ee_W);

    // joint velocity
    if (!first_state) { // to avoid jump in vel/acc in first state
      double dt = state.t_ - state.t_;
      hyq_j.qd  = (hyq_j.q  - hyq_j_prev.q)  / dt;
      hyq_j.qdd = (hyq_j.qd - hyq_j_prev.qd) / dt;
    }

    trajectory_joints.push_back(hyq_j);
    hyq_j_prev = hyq_j;
    first_state = false;
  }






//  HyqState hyq_j_prev;
//  bool first_state = true;
//  double t=0.0;
//  while (t<GetTotalTime()) {
//
//    HyqState hyq_j;
//    hyq_j.base_.lin     = GetCurrPosition(t);
//    hyq_j.base_.ang     = GetCurrOrientation(t);
//
//    FeetArray feet_W_;
//    FillCurrFeet(t, feet_W_, hyq_j.swingleg_);
//    HyqState::PosEE ee_W = {feet_W_[LF].p, feet_W_[RF].p, feet_W_[LH].p, feet_W_[RH].p};
//
//    // joint position through inverse kinematics
//    hyq_j.SetJointAngles(ee_W);
//
//    // joint velocity
//    if (!first_state) { // to avoid jump in vel/acc in first state
//      hyq_j.qd  = (hyq_j.q  - hyq_j_prev.q)  / kDiscretizationTime;
//      hyq_j.qdd = (hyq_j.qd - hyq_j_prev.qd) / kDiscretizationTime;
//    }
//
//    trajectory_joints.push_back(hyq_j);
//    hyq_j_prev = hyq_j;
//    first_state = false;
//    t += kDiscretizationTime;
//  }

  return trajectory_joints;
}

} /* namespace hyq */
} /* namespace xpp */
