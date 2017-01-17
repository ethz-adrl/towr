/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/opt/wb_traj_generator.h>
#include <kindr/rotations/Rotation.hpp>

namespace xpp {
namespace opt {

using namespace xpp::utils;


WBTrajGenerator::WBTrajGenerator()
{
  leg_lift_height_ = 0.0;
}

WBTrajGenerator::~WBTrajGenerator()
{
}

void
WBTrajGenerator::Init (const PhaseVec& phase_info, const ComMotionS& com_spline,
                       const VecFoothold& footholds, double des_height,
                       const SplineNode& curr_state, double lift_height)
{
  // get endeffector size from current node
  kNEE = curr_state.feet_W_.GetEECount();
  leg_lift_height_ = lift_height;
  com_motion_ = com_spline;

  nodes_ = BuildNodeSequence(curr_state, phase_info, footholds, des_height);

  CreateAllSplines();
}

 WBTrajGenerator::ArtiRobVec
WBTrajGenerator::BuildNodeSequence(const SplineNode& current_state,
                                   const PhaseVec& phase_info,
                                   const VecFoothold& footholds,
                                   double des_robot_height)
{
  std::vector<SplineNode> nodes;

  SplineNode prev_node = current_state;
  prev_node.t_global_ = 0.0; // always reset time to start at t=0
  nodes.push_back(prev_node);

  for (const auto& curr_phase : phase_info)
  {
    // starting point is previous state
    SplineNode goal_node = prev_node;
    goal_node.is_contact_.SetAll(true);

    for (auto c : curr_phase.swing_goal_contacts_) {
      goal_node.feet_W_.At(c.ee).p.x() = footholds.at(c.id).x();
      goal_node.feet_W_.At(c.ee).p.y() = footholds.at(c.id).y();
      goal_node.feet_W_.At(c.ee).p.z() = 0.0;
      goal_node.is_contact_.At(c.ee) = false;
      // vel, acc of endeffector always zero at nodes
    }

    // adjust roll, pitch, yaw depending on footholds
    kindr::EulerAnglesXyzPD yprIB(0.0, 0.0, 0.0);
    kindr::RotationQuaternionPD qIB(yprIB);
    goal_node.base_.ang.q = qIB.toImplementation();

    // adjust global z position of body depending on footholds
    goal_node.base_.lin.p.z() = des_robot_height + goal_node.GetZAvg();

    goal_node.t_global_ += curr_phase.duration_; // time to reach this node
    goal_node.percent_phase_ = 0.0;

    nodes.push_back(goal_node);
    prev_node = goal_node;
  }

  return nodes;
}

void WBTrajGenerator::CreateAllSplines()
{
  z_spliner_.clear();
  ori_spliner_.clear();
  ee_spliner_.clear();

  SplinerOri ori;
  ZPolynomial z_height;
  EESplinerArray feet(kNEE);

  for (int n=1; n<nodes_.size(); ++n) {
    SplineNode from = nodes_.at(n-1);
    SplineNode to   = nodes_.at(n);

    BuildPhase(from, to, z_height, ori, feet);

    z_spliner_.push_back(z_height);
    ori_spliner_.push_back(ori);

    // zmp_ remove this
//    if (n==1) // in first polynomial, swing might be in air
//      for (EEID ee : from.feet_W_.GetEEsOrdered())
//        feet.At(ee).SetZParams(percent_swing, leg_lift_height_);
//
    ee_spliner_.push_back(feet);
  }
}

void
WBTrajGenerator::BuildPhase(const SplineNode& from, const SplineNode& to,
                                 ZPolynomial& z_poly,
                                 SplinerOri& ori,
                                 EESplinerArray& feet) const
{
  double t_phase = to.t_global_ - from.t_global_;
  z_poly.SetBoundary(t_phase, from.base_.lin.Get1d(Z), to.base_.lin.Get1d(Z));

  xpp::utils::StateLin3d rpy_from, rpy_to;
  rpy_from.p = TransformQuatToRpy(from.base_.ang.q);
  rpy_to.p   = TransformQuatToRpy(to.base_.ang.q);
  ori.SetBoundary(t_phase, rpy_from, rpy_to);


  for (EEID ee : from.feet_W_.GetEEsOrdered()) {
    feet.At(ee).SetDuration(t_phase);
    feet.At(ee).SetXYParams(from.feet_W_.At(ee).Get2D(), to.feet_W_.At(ee).Get2D());
    feet.At(ee).SetZParams(from.percent_phase_, leg_lift_height_);
  }
}

Eigen::Vector3d
WBTrajGenerator::TransformQuatToRpy(const Eigen::Quaterniond& q)
{
  // wrap orientation
  kindr::RotationQuaternionPD qIB(q);

  kindr::EulerAnglesXyzPD rpyIB(qIB);
  rpyIB.setUnique(); // wrap euler angles yaw from -pi..pi

  // if yaw jumped over range from -pi..pi
  static double yaw_prev = 0.0;
  static int counter360 = 0;
  if (rpyIB.yaw()-yaw_prev < -M_PI_2) {
    std::cout << "passed yaw=0.9pi->-0.9pi, increasing counter...\n";
    counter360 += 1;
  }
  if (rpyIB.yaw()-yaw_prev > M_PI_2) {
    std::cout << "passed yaw=-0.9pi->0.9pi, decreasing counter...\n";
    counter360 -= 1;
  }
  yaw_prev = rpyIB.yaw();

  // contains information that orientation went 360deg around
  kindr::EulerAnglesXyzPD yprIB_full = rpyIB;
  yprIB_full.setYaw(rpyIB.yaw() + counter360*2*M_PI);

  return yprIB_full.toImplementation();
}

void
WBTrajGenerator::FillZState(double t_global, State3d& pos) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);

  utils::StateLin1d z_splined;
  z_spliner_.at(spline).GetPoint(t_local, z_splined);
  pos.SetDimension(z_splined, Z);
}

WBTrajGenerator::State3d
WBTrajGenerator::GetCurrPosition(double t_global) const
{
  State3d pos;

  xpp::utils::StateLin2d xy_optimized = com_motion_->GetCom(t_global);
  pos.p.topRows(kDim2d) = xy_optimized.p;
  pos.v.topRows(kDim2d) = xy_optimized.v;
  pos.a.topRows(kDim2d) = xy_optimized.a;

  FillZState(t_global, pos);
  return pos;
}

WBTrajGenerator::StateAng3d
WBTrajGenerator::GetCurrOrientation(double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);

  State3d ori_rpy;
  ori_spliner_.at(spline).GetPoint(t_local, ori_rpy);

  xpp::utils::StateAng3d ori;
  kindr::EulerAnglesXyzPD yprIB(ori_rpy.p);
  kindr::RotationQuaternionPD qIB(yprIB);
  ori.q = qIB.toImplementation();
  ori.v = ori_rpy.v;
  ori.a = ori_rpy.a;

  return ori;
}

WBTrajGenerator::FeetArray
WBTrajGenerator::GetCurrEndeffectors (double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);
  int  goal_node = spline+1;

  FeetArray feet = nodes_.at(goal_node).feet_W_;

  for (EEID ee : feet.GetEEsOrdered())
    if(!nodes_.at(goal_node).is_contact_.At(ee)) // only spline swinglegs
      feet.At(ee) = ee_spliner_.at(spline).At(ee).GetState(t_local);

  return feet;
}

WBTrajGenerator::ContactArray
WBTrajGenerator::GetCurrContactState (double t_global) const
{
  int  spline    = GetPhaseID(t_global);
  int  goal_node = spline+1;

  return nodes_.at(goal_node).is_contact_;
}

double WBTrajGenerator::GetTotalTime() const
{
  return nodes_.back().t_global_;
}

double WBTrajGenerator::GetLocalPhaseTime(double t_global) const
{
  for (int i=0; i<nodes_.size(); ++i)
    if (nodes_.at(i).t_global_ > t_global)
      return t_global - nodes_.at(i-1).t_global_;
}

int WBTrajGenerator::GetPhaseID(double t_global) const
{
  assert(t_global <= GetTotalTime()); // time inside the time frame

  for (int i=0; i<nodes_.size(); ++i)
    if (nodes_.at(i).t_global_ > t_global)
      return i-1;
}

WBTrajGenerator::ArtiRobVec
WBTrajGenerator::BuildWholeBodyTrajectory (double dt) const
{
  ArtiRobVec trajectory;

  double t=0.0;
  while (t<GetTotalTime()) {

    SplineNode state(kNEE);
    state.base_.lin     = GetCurrPosition(t);
    state.base_.ang     = GetCurrOrientation(t);
    state.feet_W_       = GetCurrEndeffectors(t);
    state.is_contact_   = GetCurrContactState(t);
    state.percent_phase_= GetPercentOfPhase(t);
    state.t_global_     = t;
    trajectory.push_back(state);

    t += dt;
  }

  return trajectory;
}

double
WBTrajGenerator::GetPercentOfPhase (double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int phase      = GetPhaseID(t_global);
  int goal_node  = phase+1;
  double t_phase = nodes_.at(goal_node).t_global_ - nodes_.at(goal_node-1).t_global_;

  return t_local/t_phase;
}

} // namespace opt
} // namespace xpp
