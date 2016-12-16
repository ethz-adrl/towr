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
  SetParams(0.0, 0.0, 0.0, 0.0);
}


WBTrajGenerator::~WBTrajGenerator()
{
}


void WBTrajGenerator::SetParams(double upswing,
               double lift_height,
               double outward_swing_distance,
               double discretization_time)
{
  kUpswingPercent = upswing;
  kLiftHeight = lift_height;
  kOutwardSwingDistance = outward_swing_distance;
  kDiscretizationTime = discretization_time;
}


void
WBTrajGenerator::Init (const PhaseVec& phase_info, const ComMotionS& com_spline,
                             const VecFoothold& footholds, double des_height,
                             const SplineNode& curr_state)
{
  // get endeffector size from current node
  kNEE = curr_state.feet_W_.size();
  feet_spliner_up_.resize(kNEE);
  feet_spliner_down_.resize(kNEE);



  nodes_ = BuildNodeSequence(curr_state, phase_info, footholds, des_height);
  CreateAllSplines(nodes_);
  com_motion_ = com_spline;
}


 WBTrajGenerator::ArtiRobVec
WBTrajGenerator::BuildNodeSequence(const SplineNode& P_init,
                                         const PhaseVec& phase_info,
                                         const VecFoothold& footholds,
                                         double des_robot_height)
{
  std::vector<SplineNode> nodes;

  SplineNode prev_node = P_init;
  nodes.push_back(prev_node);

  for (const auto& curr_phase : phase_info)
  {
    // starting point is previous state
    SplineNode goal_node = prev_node;
    std::fill(goal_node.swingleg_.begin(), goal_node.swingleg_.end(), false);

    for (auto c : curr_phase.swing_goal_contacts_) {
      int ee = static_cast<int>(c.ee);
      goal_node.feet_W_.at(ee).p.x() = footholds.at(c.id).x();
      goal_node.feet_W_.at(ee).p.y() = footholds.at(c.id).y();
      goal_node.feet_W_.at(ee).p.z() = 0.0;
      goal_node.swingleg_[ee] = true;
    }

    // adjust roll, pitch, yaw depending on footholds
    kindr::EulerAnglesXyzPD yprIB(0.0, 0.0, 0.0);
    kindr::RotationQuaternionPD qIB(yprIB);
    goal_node.base_.ang.q = qIB.toImplementation();

    // adjust global z position of body depending on footholds
    goal_node.base_.lin.p.z() = des_robot_height + goal_node.GetZAvg();

    goal_node.T = curr_phase.duration_; // time to reach this node

    nodes.push_back(goal_node);
    prev_node = goal_node;
  }

  return nodes;
}


void WBTrajGenerator::CreateAllSplines(const std::vector<SplineNode>& nodes)
{
  z_spliner_.clear();
  ori_spliner_.clear();
  feet_spliner_up_.clear();
  feet_spliner_down_.clear();

  SplinerOri ori;
  ZPolynomial z_height;

  FeetSplinerArray feet_up(kNEE), feet_down(kNEE);
  for (int n=1; n<nodes_.size(); ++n) {
    SplineNode from = nodes.at(n-1);
    SplineNode to   = nodes.at(n);

    BuildOneSegment(from, to, z_height, ori, feet_up, feet_down);
    z_spliner_.push_back(z_height);
    ori_spliner_.push_back(ori);
    feet_spliner_up_.push_back(feet_up);
    feet_spliner_down_.push_back(feet_down);
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
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

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
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

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
  FeetArray feet;

  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);
  int  goal_node = spline+1;

  feet = nodes_.at(goal_node).feet_W_;

  for (int leg=0; leg<feet.size(); ++leg) {
    if(nodes_.at(goal_node).swingleg_[leg]) { // only spline swinglegs

      double t_upswing = nodes_.at(goal_node).T * kUpswingPercent;

      if ( t_local < t_upswing) // leg swinging up
        feet_spliner_up_.at(spline)[leg].GetPoint(t_local, feet[leg]);
      else // leg swinging down
        feet_spliner_down_.at(spline)[leg].GetPoint(t_local - t_upswing, feet[leg]);
    }
  }

  return feet;
}

WBTrajGenerator::ContactArray
WBTrajGenerator::GetCurrContactState (double t_global) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);
  int  goal_node = spline+1;

  return nodes_.at(goal_node).swingleg_;
}

void
WBTrajGenerator::BuildOneSegment(const SplineNode& from, const SplineNode& to,
                                 ZPolynomial& z_poly,
                                 SplinerOri& ori,
                                 FeetSplinerArray& feet_up,
                                 FeetSplinerArray& feet_down) const
{
  z_poly.SetBoundary(to.T, from.base_.lin.Get1d(Z), to.base_.lin.Get1d(Z));
  ori = BuildOrientationRpySpline(from, to);
  feet_up = BuildFootstepSplineUp(from, to);

  // this is the outter/upper-most point the foot swings to
  FeetArray f_switch(kNEE);
  double t_switch = to.T * kUpswingPercent;
  for (int leg=0; leg<feet_up.size(); ++leg) {
     feet_up[leg].GetPoint(t_switch, f_switch[leg]);
  }

  feet_down = BuildFootstepSplineDown(f_switch, to);
}

WBTrajGenerator::SplinerOri
WBTrajGenerator::BuildOrientationRpySpline(const SplineNode& from, const SplineNode& to) const
{
  xpp::utils::StateLin3d rpy_from, rpy_to;
  rpy_from.p = TransformQuatToRpy(from.base_.ang.q);
  rpy_to.p   = TransformQuatToRpy(to.base_.ang.q);

  SplinerOri ori;
  ori.SetBoundary(to.T, rpy_from, rpy_to);
  return ori;
}

WBTrajGenerator::FeetSplinerArray
WBTrajGenerator::BuildFootstepSplineUp(const SplineNode& from, const SplineNode& to) const
{
  FeetSplinerArray feet_up(kNEE);

  // Feet spliner for all legs, even if might be stance legs
  for (int leg=0; leg<from.feet_W_.size(); ++leg) {

    // raise intermediate foothold dependant on foothold difference
    double delta_z = std::abs(to.feet_W_[leg].p.z() - from.feet_W_[leg].p.z());
    State3d foot_raised = to.feet_W_[leg];
    foot_raised.p.z() += kLiftHeight + delta_z;

    // move outward only if footholds significantly differ in height
    if (delta_z > 0.01) {
      int sign = (leg == 0 || leg == 2) ? 1 : -1;
      foot_raised.p.y() += sign * kOutwardSwingDistance;
    }

    // upward swing
    double swing_time = to.T;
    feet_up[leg].SetBoundary(swing_time, from.feet_W_[leg], foot_raised);
  }

  return feet_up;
}

WBTrajGenerator::FeetSplinerArray
WBTrajGenerator::BuildFootstepSplineDown(const FeetArray& feet_at_switch,
                                    const SplineNode& to) const
{
  FeetSplinerArray feet_down(kNEE);

  // Feet spliner for all legs, even if might be stance legs
  for (int leg=0; leg<to.feet_W_.size(); ++leg) {

    // downward swing from the foothold at switch state to original
    double duration = to.T * (1.0-kUpswingPercent);
    feet_down[leg].SetBoundary(duration, feet_at_switch[leg], to.feet_W_[leg]);
  }

  return feet_down;
}

double WBTrajGenerator::GetTotalTime() const
{
  double T = 0.0;
  for (uint n = 1; n < nodes_.size(); n++) {
    T += nodes_.at(n).T;
  }
  return T;
}

double WBTrajGenerator::GetLocalSplineTime(double t_global) const
{
  int spline = GetSplineID(t_global);
  int goal_node = spline+1;

  double t_local = t_global;
  for (int n = 1; n < goal_node; n++) {
    t_local -= nodes_.at(n).T;
  }
  return t_local;
}

int WBTrajGenerator::GetSplineID(double t) const
{
  assert(t <= GetTotalTime()); // time inside the time frame

  double t_junction = 0.0;
  for (int n=1; n<nodes_.size(); ++n) {
    t_junction += nodes_.at(n).T;

    if (t <= t_junction + 1e-4) // so at "equal", previous spline is returned
      return n-1; // since first spline connects node 0 and 1
  }
  assert(false); // this should never be reached
  return -1;
}

WBTrajGenerator::ArtiRobVec
WBTrajGenerator::BuildWholeBodyTrajectory () const
{
  ArtiRobVec trajectory;

  double t=0.0;
  while (t<GetTotalTime()) {

    SplineNode state(kNEE);
    state.base_.lin     = GetCurrPosition(t);
    state.base_.ang     = GetCurrOrientation(t);
    state.feet_W_       = GetCurrEndeffectors(t);
    state.swingleg_     = GetCurrContactState(t);
    state.T = t;
    trajectory.push_back(state);

    t += kDiscretizationTime;
  }

  return trajectory;
}

} // namespace opt
} // namespace xpp
