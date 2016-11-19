/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/hyq/hyq_spliner.h>
#include <kindr/rotations/Rotation.hpp>
#include <xpp/hyq/hyq_inverse_kinematics.h>

namespace xpp {
namespace hyq {

using namespace xpp::utils;

HyqSpliner::HyqSpliner()
{
  SetParams(0.0, 0.0, 0.0, 0.0);
}

HyqSpliner::~HyqSpliner()
{
}

void HyqSpliner::SetParams(double upswing,
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
HyqSpliner::Init (const xpp::opt::PhaseVec& phase_info, const ComSpline& com_spline,
                  const VecFoothold& contacts, double des_height,
                  const HyqState& curr_state)
{
  nodes_.clear();
  pos_spliner_.clear();
  ori_spliner_.clear();
  optimized_xy_spline_.clear();
  feet_spliner_down_.clear();
  feet_spliner_up_.clear();

  nodes_ = BuildNodeSequence(curr_state, phase_info, contacts, des_height);
  CreateAllSplines(nodes_);
  optimized_xy_spline_ = com_spline;
}

std::vector<SplineNode>
HyqSpliner::BuildNodeSequence(const HyqState& P_init,
                              const xpp::opt::PhaseVec& phase_info,
                              const VecFoothold& footholds,
                              double des_robot_height)
{
  std::vector<SplineNode> nodes;

  SplineNode prev_node(P_init, 0.0); // this node has to be reached instantly (t=0)
  nodes.push_back(prev_node);

  for (const auto& curr_phase : phase_info)
  {
    // copy a few values from previous state
    SplineNode goal_node = prev_node;
    goal_node.swingleg_ = false;

    // add the desired foothold after swinging if this is a step phase
    if (curr_phase.IsStep()) {
      int step = curr_phase.n_completed_steps_;
      LegID step_leg = footholds.at(step).leg;
      goal_node.feet_W_[step_leg].p   = footholds.at(step).p;
      goal_node.swingleg_[step_leg] = true;
    }

    // adjust roll, pitch, yaw depending on footholds
    kindr::EulerAnglesXyzPD yprIB(0.0, 0.0, 0.0);
    kindr::RotationQuaternionPD qIB(yprIB);
    goal_node.base_.ang.q = qIB.toImplementation();

    // adjust global z position of body depending on footholds
    goal_node.base_.lin.p(Z) = des_robot_height + goal_node.GetZAvg(); // height of footholds

    goal_node.T = curr_phase.duration_; // time to reach this node

    nodes.push_back(goal_node);
    prev_node = goal_node;
  }

  return nodes;
}

void HyqSpliner::CreateAllSplines(const std::vector<SplineNode>& nodes)
{
  pos_spliner_.clear();
  ori_spliner_.clear();
  feet_spliner_up_.clear();
  feet_spliner_down_.clear();

  Spliner3d pos, ori;
  LegDataMap< Spliner3d > feet_up, feet_down;
  for (int n=1; n<nodes_.size(); ++n) {
    SplineNode from = nodes.at(n-1);
    SplineNode to   = nodes.at(n);

    BuildOneSegment(from, to, pos, ori, feet_up, feet_down);
    pos_spliner_.push_back(pos);
    ori_spliner_.push_back(ori);
    feet_spliner_up_.push_back(feet_up);
    feet_spliner_down_.push_back(feet_down);
  }
}

Eigen::Vector3d
HyqSpliner::TransformQuatToRpy(const Eigen::Quaterniond& q)
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
HyqSpliner::FillZState(double t_global, Point& pos) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

  Point pos_splined;
  pos_spliner_.at(spline).GetPoint(t_local, pos_splined);

  pos.p.z() = pos_splined.p.z();
  pos.v.z() = pos_splined.v.z();
  pos.a.z() = pos_splined.a.z();
}

HyqSpliner::Point
HyqSpliner::GetCurrPosition(double t_global) const
{
  Point pos;

  xpp::utils::BaseLin2d xy_optimized = ComPolynomialHelpers::GetCOM(t_global, optimized_xy_spline_);
  pos.p.topRows(kDim2d) = xy_optimized.p;
  pos.v.topRows(kDim2d) = xy_optimized.v;
  pos.a.topRows(kDim2d) = xy_optimized.a;

  FillZState(t_global, pos);

  return pos;
}

xpp::utils::BaseAng3d
HyqSpliner::GetCurrOrientation(double t_global) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

  Point ori_rpy;
  ori_spliner_.at(spline).GetPoint(t_local, ori_rpy);

  xpp::utils::BaseAng3d ori;
  kindr::EulerAnglesXyzPD yprIB(ori_rpy.p);
  kindr::RotationQuaternionPD qIB(yprIB);
  ori.q = qIB.toImplementation();
  ori.v = ori_rpy.v;
  ori.a = ori_rpy.a;

  return ori;
}

void
HyqSpliner::FillCurrFeet(double t_global,
                        LegDataMap<Point>& feet,
                        LegDataMap<bool>& swingleg) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);
  int  goal_node = spline+1;

  swingleg = false;

  feet = nodes_.at(goal_node).feet_W_;

  for (int leg=0; leg<4; ++leg) {
    if(nodes_.at(goal_node).swingleg_[leg]) { // only spline swinglegs

      double t_upswing = nodes_.at(goal_node).T * kUpswingPercent;

      if ( t_local < t_upswing) // leg swinging up
        feet_spliner_up_.at(spline)[leg].GetPoint(t_local, feet[leg]);
      else // leg swinging down
        feet_spliner_down_.at(spline)[leg].GetPoint(t_local - t_upswing, feet[leg]);

      swingleg[leg] = true;
    }
  }
}

void HyqSpliner::BuildOneSegment(const SplineNode& from, const SplineNode& to,
                                 Spliner3d& pos, Spliner3d& ori,
                                 LegDataMap< Spliner3d >& feet_up,
                                 LegDataMap< Spliner3d >& feet_down) const
{
  pos = BuildPositionSpline(from, to);
  ori = BuildOrientationRpySpline(from, to);
  feet_up = BuildFootstepSplineUp(from, to);

  // this is the outter/upper-most point the foot swings to
  LegDataMap<Point> f_switch;
  double t_switch = to.T * kUpswingPercent;
  for (LegID leg : LegIDArray) {
     feet_up[leg].GetPoint(t_switch, f_switch[leg]);
  }

  feet_down = BuildFootstepSplineDown(f_switch, to);
}

HyqSpliner::Spliner3d
HyqSpliner::BuildPositionSpline(const SplineNode& from, const SplineNode& to) const
{
  Spliner3d pos;
  pos.SetBoundary(to.T, from.base_.lin, to.base_.lin);
  return pos;
}

HyqSpliner::Spliner3d
HyqSpliner::BuildOrientationRpySpline(const SplineNode& from, const SplineNode& to) const
{
  xpp::utils::BaseLin3d rpy_from, rpy_to;
  rpy_from.p = TransformQuatToRpy(from.base_.ang.q);
  rpy_to.p   = TransformQuatToRpy(to.base_.ang.q);

  Spliner3d ori;
  ori.SetBoundary(to.T, rpy_from, rpy_to);
  return ori;
}

xpp::hyq::LegDataMap<HyqSpliner::Spliner3d>
HyqSpliner::BuildFootstepSplineUp(const SplineNode& from, const SplineNode& to) const
{
  LegDataMap< Spliner3d > feet_up;

  // Feet spliner for all legs, even if might be stance legs
  for (LegID leg : LegIDArray) {

    // raise intermediate foothold dependant on foothold difference
    double delta_z = std::abs(to.feet_W_[leg].p.z() - from.feet_W_[leg].p.z());
    Point foot_raised = to.feet_W_[leg];
    foot_raised.p.z() += kLiftHeight + delta_z;

    // move outward only if footholds significantly differ in height
    if (delta_z > 0.01) {
      int sign = (leg == LH || leg == LF) ? 1 : -1;
      foot_raised.p.y() += sign * kOutwardSwingDistance;
    }

    // upward swing
    double swing_time = to.T;
    feet_up[leg].SetBoundary(swing_time, from.feet_W_[leg], foot_raised);
  }

  return feet_up;
}

xpp::hyq::LegDataMap<HyqSpliner::Spliner3d>
HyqSpliner::BuildFootstepSplineDown(const LegDataMap<Point>& feet_at_switch,
                                    const SplineNode& to) const
{
  LegDataMap< Spliner3d > feet_down;

  // Feet spliner for all legs, even if might be stance legs
  for (LegID leg : LegIDArray) {

    // downward swing from the foothold at switch state to original
    double duration = to.T * (1.0-kUpswingPercent);
    feet_down[leg].SetBoundary(duration, feet_at_switch[leg], to.feet_W_[leg]);
  }

  return feet_down;
}

double HyqSpliner::GetTotalTime() const
{
  double T = 0.0;
  for (uint n = 1; n < nodes_.size(); n++) {
    T += nodes_.at(n).T;
  }
  return T;
}

double HyqSpliner::GetLocalSplineTime(double t_global) const
{
  int spline = GetSplineID(t_global);
  int goal_node = spline+1;

  double t_local = t_global;
  for (int n = 1; n < goal_node; n++) {
    t_local -= nodes_.at(n).T;
  }
  return t_local;
}

int HyqSpliner::GetSplineID(double t) const
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

SplineNode::SplineNode (const HyqState& state_joints, double t_max)
{
  swingleg_ = state_joints.swingleg_;
  base_ = state_joints.base_;

  LegDataMap<BaseLin3d> feet;
  auto ee_W = state_joints.GetEEInWorld();

  for (int leg=0; leg<ee_W.size(); ++leg){
    feet_W_[leg].p = ee_W[leg];
    feet_W_[leg].v.setZero();
    feet_W_[leg].a.setZero();
  }

  T = t_max;
}

std::array<Eigen::Vector3d, kNumSides>
SplineNode::GetAvgSides() const
{
  typedef std::pair <Side,Side> LegSide;
  static LegDataMap<LegSide> leg_sides;

  leg_sides[LF] = LegSide( LEFT_SIDE, FRONT_SIDE);
  leg_sides[RF] = LegSide(RIGHT_SIDE, FRONT_SIDE);
  leg_sides[LH] = LegSide( LEFT_SIDE,  HIND_SIDE);
  leg_sides[RH] = LegSide(RIGHT_SIDE,  HIND_SIDE);

  std::array<Vector3d, kNumSides> pos_avg;
  for (Side s : SideArray) pos_avg[s] = Vector3d::Zero(); // zero values

  for (LegID leg : LegIDArray)
  {
    pos_avg[leg_sides[leg].first]  += feet_W_[leg].p;
    pos_avg[leg_sides[leg].second] += feet_W_[leg].p;
  }

  for (Side s : SideArray)
    pos_avg[s] /= std::tuple_size<LegSide>::value; // 2 feet per side
  return pos_avg;
}

double SplineNode::GetZAvg() const
{
  std::array<Vector3d, kNumSides> avg = GetAvgSides();
  return (avg[FRONT_SIDE](Z) + avg[HIND_SIDE](Z)) / 2;
}

HyqSpliner::SplineNodeVec
HyqSpliner::GetInterpolatedNodes () const
{
  SplineNodeVec trajectory;

  double t=0.0;
  while (t<GetTotalTime()) {

    SplineNode state;

    state.base_.lin = GetCurrPosition(t);
    state.base_.ang = GetCurrOrientation(t);
    FillCurrFeet(t, state.feet_W_, state.swingleg_);

    trajectory.push_back(state);

    t += kDiscretizationTime;
  }

  return trajectory;
}

HyqSpliner::HyqStateVec
HyqSpliner::BuildWholeBodyTrajectoryJoints () const
{
  auto trajectory_ee = GetInterpolatedNodes();
  HyqStateVec trajectory_joints;
  HyqInverseKinematics inv_kin;

  HyqState hyq_j_prev;
  bool first_state = true;
  for (auto hyq : trajectory_ee) {

    HyqState hyq_j;
    hyq_j.base_     = hyq.base_;
    hyq_j.swingleg_ = hyq.swingleg_;

    // add joint position
    HyqState::PosEE ee_W = {hyq.feet_W_[LF].p, hyq.feet_W_[RF].p, hyq.feet_W_[LH].p, hyq.feet_W_[RH].p};
    hyq_j.SetJointAngles(ee_W);

    // joint velocity
    if (!first_state) { // to avoid jump in vel/acc in first state
      hyq_j.qd  = (hyq_j.q  - hyq_j_prev.q)  / kDiscretizationTime;
      hyq_j.qdd = (hyq_j.qd - hyq_j_prev.qd) / kDiscretizationTime;
    }

    trajectory_joints.push_back(hyq_j);
    hyq_j_prev = hyq_j;
    first_state = false;
  }

  return trajectory_joints;
}

} // namespace hyq
} // namespace xpp
