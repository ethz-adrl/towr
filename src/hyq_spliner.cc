/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/hyq/hyq_spliner.h>
#include <kindr/rotations/Rotation.hpp>

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
                  const VecFoothold& footholds, double des_height,
                  const HyqState& curr_state)
{
  nodes_ = BuildNodeSequence(curr_state, phase_info, footholds, des_height);
  CreateAllSplines(nodes_);
  optimized_xy_spline_ = com_spline;
}

std::vector<SplineNode>
HyqSpliner::BuildNodeSequence(const HyqState& P_init,
                              const xpp::opt::PhaseVec& phase_info,
                              const VecFoothold& footholds, // zmp_ make this only std::vec2d
                              double des_robot_height)
{
  std::vector<SplineNode> nodes;

  SplineNode prev_node(P_init, 0.0); // this node has to be reached instantly (t=0)
  nodes.push_back(prev_node);

  for (const auto& curr_phase : phase_info)
  {
    // starting point is previous state
    SplineNode goal_node = prev_node;
    goal_node.swingleg_ = false;

    for (auto c : curr_phase.swing_goal_contacts_) {
      LegID ee = static_cast<LegID>(c.ee);
      goal_node.feet_W_[ee].p.x() = footholds.at(c.id).x();
      goal_node.feet_W_[ee].p.y() = footholds.at(c.id).y();
      goal_node.feet_W_[ee].p.z() = 0.0;
      goal_node.swingleg_[ee] = true;
    }

    // adjust roll, pitch, yaw depending on footholds
    kindr::EulerAnglesXyzPD yprIB(0.0, 0.0, 0.0);
    kindr::RotationQuaternionPD qIB(yprIB);
    goal_node.base_ang_.q = qIB.toImplementation();

    // adjust global z position of body depending on footholds
    goal_node.base_z_.p = des_robot_height + goal_node.GetZAvg();

    goal_node.T = curr_phase.duration_; // time to reach this node

    nodes.push_back(goal_node);
    prev_node = goal_node;
  }

  return nodes;
}

void HyqSpliner::CreateAllSplines(const std::vector<SplineNode>& nodes)
{
  z_spliner_.clear();
  ori_spliner_.clear();
  feet_spliner_up_.clear();
  feet_spliner_down_.clear();

  SplinerOri ori;
  ZPolynomial z_height;

  LegDataMap< SplinerFeet > feet_up, feet_down;
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
HyqSpliner::FillZState(double t_global, State1d& pos) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

  utils::StateLin1d z_splined;
  z_spliner_.at(spline).GetPoint(t_local, z_splined);
  pos.SetDimension(z_splined, Z);
}

HyqSpliner::State1d
HyqSpliner::GetCurrPosition(double t_global) const
{
  State1d pos;

  xpp::utils::StateLin2d xy_optimized = optimized_xy_spline_->GetCom(t_global);
  pos.p.topRows(kDim2d) = xy_optimized.p;
  pos.v.topRows(kDim2d) = xy_optimized.v;
  pos.a.topRows(kDim2d) = xy_optimized.a;

  FillZState(t_global, pos);
  return pos;
}

xpp::utils::StateAng3d
HyqSpliner::GetCurrOrientation(double t_global) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);

  State1d ori_rpy;
  ori_spliner_.at(spline).GetPoint(t_local, ori_rpy);

  xpp::utils::StateAng3d ori;
  kindr::EulerAnglesXyzPD yprIB(ori_rpy.p);
  kindr::RotationQuaternionPD qIB(yprIB);
  ori.q = qIB.toImplementation();
  ori.v = ori_rpy.v;
  ori.a = ori_rpy.a;

  return ori;
}

void
HyqSpliner::FillCurrFeet(double t_global,
                        LegDataMap<State1d>& feet,
                        LegDataMap<bool>& swingleg) const
{
  double t_local = GetLocalSplineTime(t_global);
  int  spline    = GetSplineID(t_global);
  int  goal_node = spline+1;

  swingleg = false;

  feet = nodes_.at(goal_node).feet_W_;

  for (auto leg : {LF, RF, LH, RH}) {
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
                                 ZPolynomial& z_poly,
                                 SplinerOri& ori,
                                 LegDataMap< SplinerFeet >& feet_up,
                                 LegDataMap< SplinerFeet >& feet_down) const
{
  z_poly.SetBoundary(to.T, from.base_z_, to.base_z_);
  ori = BuildOrientationRpySpline(from, to);
  feet_up = BuildFootstepSplineUp(from, to);

  // this is the outter/upper-most point the foot swings to
  LegDataMap<State1d> f_switch;
  double t_switch = to.T * kUpswingPercent;
  for (LegID leg : LegIDArray) {
     feet_up[leg].GetPoint(t_switch, f_switch[leg]);
  }

  feet_down = BuildFootstepSplineDown(f_switch, to);
}

HyqSpliner::SplinerOri
HyqSpliner::BuildOrientationRpySpline(const SplineNode& from, const SplineNode& to) const
{
  xpp::utils::StateLin3d rpy_from, rpy_to;
  rpy_from.p = TransformQuatToRpy(from.base_ang_.q);
  rpy_to.p   = TransformQuatToRpy(to.base_ang_.q);

  SplinerOri ori;
  ori.SetBoundary(to.T, rpy_from, rpy_to);
  return ori;
}

xpp::hyq::LegDataMap<HyqSpliner::SplinerFeet>
HyqSpliner::BuildFootstepSplineUp(const SplineNode& from, const SplineNode& to) const
{
  LegDataMap< SplinerFeet > feet_up;

  // Feet spliner for all legs, even if might be stance legs
  for (LegID leg : LegIDArray) {

    // raise intermediate foothold dependant on foothold difference
    double delta_z = std::abs(to.feet_W_[leg].p.z() - from.feet_W_[leg].p.z());
    State1d foot_raised = to.feet_W_[leg];
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

xpp::hyq::LegDataMap<HyqSpliner::SplinerFeet>
HyqSpliner::BuildFootstepSplineDown(const LegDataMap<State1d>& feet_at_switch,
                                    const SplineNode& to) const
{
  LegDataMap< SplinerFeet > feet_down;

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

SplineNode::SplineNode (const HyqState& state, double t_max)
{
  swingleg_ = state.swingleg_;
  base_ang_ = state.base_.ang;
  base_z_ = state.base_.lin.Get1d(Z);

  LegDataMap<BaseLin3d> feet;
  auto W_p_ee = state.GetEEInWorld();
  auto W_v_ee = state.GetEEInVelWorld();

  for (int leg=0; leg<W_p_ee.size(); ++leg){
    feet_W_[leg].p = W_p_ee[leg];
    feet_W_[leg].v = W_v_ee[leg];
    feet_W_[leg].a.setZero(); // mpc fill this at some point as well
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

HyqSpliner::HyqStateVec
HyqSpliner::BuildWholeBodyTrajectoryJoints () const
{
  HyqStateVec trajectory_joints;

  HyqState hyq_j_prev;
  bool first_state = true;
  double t=0.0;
  while (t<GetTotalTime()) {

    HyqState hyq_j;
    hyq_j.base_.lin     = GetCurrPosition(t);
    hyq_j.base_.ang     = GetCurrOrientation(t);

    LegDataMap<StateLin3d> feet_W_;
    FillCurrFeet(t, feet_W_, hyq_j.swingleg_);
    HyqState::PosEE ee_W = {feet_W_[LF].p, feet_W_[RF].p, feet_W_[LH].p, feet_W_[RH].p};

    // joint position through inverse kinematics
    hyq_j.SetJointAngles(ee_W);

    // joint velocity
    if (!first_state) { // to avoid jump in vel/acc in first state
      hyq_j.qd  = (hyq_j.q  - hyq_j_prev.q)  / kDiscretizationTime;
      hyq_j.qdd = (hyq_j.qd - hyq_j_prev.qd) / kDiscretizationTime;
    }

    trajectory_joints.push_back(hyq_j);
    hyq_j_prev = hyq_j;
    first_state = false;
    t += kDiscretizationTime;
  }

  return trajectory_joints;
}

} // namespace hyq
} // namespace xpp
