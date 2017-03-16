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

WBTrajGenerator::WBTrajGenerator()
{
  leg_lift_height_ = 0.0;
}

WBTrajGenerator::~WBTrajGenerator()
{
}

void
WBTrajGenerator::Init (const PhaseVec& phase_info, const ComMotionS& com_spline,
                       const VecFoothold& footholds, const EEPtr& ee_motion,
                       const SplineNode& curr_state, double lift_height,
                       const Vector3d& com_offset)
{
  // get endeffector size from current node
  kNEE = curr_state.GetEECount();
  leg_lift_height_ = lift_height;
  com_motion_ = com_spline;
  offset_geom_to_com_ = com_offset;
  ee_spliner_ = ee_motion;

  t_start_ = curr_state.GetTime();
  phase_start_ = curr_state.GetCurrentPhase();
  nodes_.push_back(curr_state);
  nodes_.back().SetTime(0.0); // internally, the motion starts at t=0

  BuildNodeSequence(phase_info, footholds);

  CreateAllSplines();
}

void
WBTrajGenerator::BuildNodeSequence(const PhaseVec& phase_info,
                                   const VecFoothold& footholds)
{
  int phase_id = 0;
  for (const auto& curr_phase : phase_info) {
    // starting point is previous state
    SplineNode prev_node = nodes_.back();
    SplineNode goal_node(prev_node.GetEECount());

    EEXppPos pos_W = prev_node.GetEEPos();
    for (auto c : curr_phase.swinglegs_) {
      pos_W.At(c.ee).x() = footholds.at(c.id).p.x();
      pos_W.At(c.ee).y() = footholds.at(c.id).p.y();
      pos_W.At(c.ee).z() = footholds.at(c.id).p.z();
    }

    SplineNode::ContactState contact_state(prev_node.GetEECount());
    contact_state.SetAll(false);
    for (auto c : curr_phase.GetAllContacts())
      contact_state.At(c.ee) = true;

    goal_node.SetEEState(kPos, pos_W);
    goal_node.SetContactState(contact_state);
    // vel, acc of endeffector always zero at nodes

    // adjust roll, pitch, yaw depending on footholds
    State3d base = prev_node.GetBase();
    kindr::EulerAnglesXyzPD yprIB(0.0, 0.0, 0.0);
    kindr::RotationQuaternionPD qIB(yprIB);
    base.ang.q = qIB.toImplementation();

    base.lin.p.z() = com_motion_->GetZHeight() - offset_geom_to_com_.z();// + goal_node.GetZAvg();
    goal_node.SetBase(base);

    goal_node.SetTime(prev_node.GetTime() + curr_phase.duration_); // time to reach this node
    goal_node.SetPercentPhase(0.0);
    goal_node.SetCurrentPhase(phase_id++);

    nodes_.push_back(goal_node);
  }
}

void WBTrajGenerator::CreateAllSplines()
{
  z_spliner_.clear();
  ori_spliner_.clear();

  SplinerOri ori;
  ZPolynomial z_height;

//  ee_spliner_.SetInitialPos(nodes_.front().GetEEPos());

  for (int n=1; n<nodes_.size(); ++n) {
    SplineNode from = nodes_.at(n-1);
    SplineNode to   = nodes_.at(n);

    BuildPhase(from, to, z_height, ori);

    z_spliner_.push_back(z_height);
    ori_spliner_.push_back(ori);

//    for (auto& ee : nodes_.front().GetEEPos().GetEEsOrdered()) {
//
//      double t_local = to.GetTime() - from.GetTime();
//      if (to.GetContactState().At(ee)) // endeffector in contact
//        ee_spliner_.GetMotion(ee).AddStancePhase(t_local);
//      else
//        ee_spliner_.GetMotion(ee).AddSwingPhase(t_local, to.GetEEPos().At(ee));
//    }
  }
}

void
WBTrajGenerator::BuildPhase(const SplineNode& from, const SplineNode& to,
                                 ZPolynomial& z_poly,
                                 SplinerOri& ori) const
{
  double t_phase = to.GetTime() - from.GetTime();
  z_poly.SetBoundary(t_phase, from.GetBase().lin.Get1d(Z), to.GetBase().lin.Get1d(Z));

  StateLin3d rpy_from, rpy_to;
  rpy_from.p = TransformQuatToRpy(from.GetBase().ang.q);
  rpy_to.p   = TransformQuatToRpy(to.GetBase().ang.q);
  ori.SetBoundary(t_phase, rpy_from, rpy_to);
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
WBTrajGenerator::FillZState(double t_global, StateLin3d& pos) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);

  StateLin1d z_splined;
  z_spliner_.at(spline).GetPoint(t_local, z_splined);
  pos.SetDimension(z_splined, Z);
}

State3d
WBTrajGenerator::GetCurrentBase (double t_global) const
{
  State3d base;
  base.lin = GetCurrPosition(t_global);
  base.ang = GetCurrOrientation(t_global);
//  base.ang = xpp::utils::StateAng3d();
  return base;
}

StateLin3d
WBTrajGenerator::GetCurrPosition(double t_global) const
{
  StateLin3d pos;
//  pos.p.z() = walking_height_;
  FillZState(t_global, pos);

  StateLin2d com_xy = com_motion_->GetCom(t_global);

  // since the optimized motion is for the CoM and not the geometric center
  pos.p.topRows(kDim2d) = com_xy.p - offset_geom_to_com_.topRows<kDim2d>();
  pos.v.topRows(kDim2d) = com_xy.v;
  pos.a.topRows(kDim2d) = com_xy.a;

  return pos;
}

StateAng3d
WBTrajGenerator::GetCurrOrientation(double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);

  StateLin3d ori_rpy;
  ori_spliner_.at(spline).GetPoint(t_local, ori_rpy);

  StateAng3d ori;
  kindr::EulerAnglesXyzPD yprIB(ori_rpy.p);
  kindr::RotationQuaternionPD qIB(yprIB);
  ori.q = qIB.toImplementation();
  ori.v = ori_rpy.v;
  ori.a = ori_rpy.a;

  return ori;
}

WBTrajGenerator::ContactArray
WBTrajGenerator::GetCurrContactState (double t_global) const
{
  int  spline    = GetPhaseID(t_global);
  int  goal_node = spline+1;

  return nodes_.at(goal_node).GetContactState(); // this must be pointing to first node!
}

double WBTrajGenerator::GetTotalTime() const
{
  return nodes_.back().GetTime();
}

double WBTrajGenerator::GetLocalPhaseTime(double t_global) const
{
  for (int i=0; i<nodes_.size(); ++i)
    if (nodes_.at(i).GetTime() > t_global)
      return t_global - nodes_.at(i-1).GetTime();
}

int WBTrajGenerator::GetPhaseID(double t_global) const
{
  assert(t_global <= GetTotalTime()); // time inside the time frame

  for (int i=0; i<nodes_.size(); ++i)
    if (nodes_.at(i).GetTime() > t_global)
      return i-1;
}

WBTrajGenerator::ArtiRobVec
WBTrajGenerator::BuildWholeBodyTrajectory (double dt) const
{
  ArtiRobVec trajectory;

  double t=0.0;
  double T = GetTotalTime();
  while (t<T) {
    trajectory.push_back(GetRobotState(t));
    t += dt;
  }
  trajectory.push_back(GetRobotState(T-1e-10)); // add final state

  return trajectory;
}

WBTrajGenerator::SplineNode
WBTrajGenerator::GetRobotState (double t) const
{
  SplineNode state(kNEE);
  state.SetBase(GetCurrentBase(t));
  state.SetEEState(ee_spliner_->GetEndeffectors(t));
  state.SetContactState(GetCurrContactState(t));
  state.SetPercentPhase(GetPercentOfPhase(t));
  state.SetTime(t_start_ + t); // keep track of global time
  state.SetCurrentPhase(phase_start_  + 1 + GetPhaseID(t));
  return state;
}

double
WBTrajGenerator::GetPercentOfPhase (double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int phase      = GetPhaseID(t_global);
  int goal_node  = phase+1;
  double t_phase = nodes_.at(goal_node).GetTime() - nodes_.at(goal_node-1).GetTime();

  return t_local/t_phase;
}

} // namespace opt
} // namespace xpp
