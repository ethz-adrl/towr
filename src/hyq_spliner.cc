/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/hyq/hyq_spliner.h>
#include <xpp/utils/logger_helpers-inl.h>

namespace xpp {
namespace hyq {

using ::xpp::utils::Y;
using ::xpp::utils::Z;


HyqSpliner::HyqSpliner()
{
  log_ = log4cxx::Logger::getLogger("xpp.hyq.hyqspliner");
}


void HyqSpliner::SetParams(double upswing,
               double lift_height,
               double outward_swing_distance,
               double t_4ls)
{
  kUpswingPercent = upswing;
  kLiftHeight = lift_height;
  kOutwardSwingDistance = outward_swing_distance;
  kTimeFourLeggSupp = t_4ls;
}


void HyqSpliner::AddNode(const HyqState& state, double t_max)
{
  // Orientation converted to 3D-Point (rpy, 0, 0)
  Eigen::Vector3d rpy;
  xpp::utils::Orientation::QuaternionToRPY(state.base_.ori.q, rpy);
  Point ori = Point(rpy); // Fixme: this sets zero angular velocity and acceleration

  nodes_.push_back(SplineNode(state.base_.pos, ori, state.feet_, state.SwinglegID(), t_max));
}


HyqState HyqSpliner::getPoint(double t_global)
{

  HyqState curr;

  /** Transform global time to local spline time dt */
  double dt = t_global;
  for (uint n = 1; n < curr_goal_; n++) {
    dt -= nodes_[n].T;
  }
  if (dt < 0.0) dt = 0.0; // negative numbers are not splined properly

  /** Positions */
  pos_spliner_.GetPoint(dt, curr.base_.pos);

  /** Orientations */
  Point curr_ori, curr_foot;
  ori_spliner_.GetPoint(dt, curr_ori);
  curr.base_.ori.q = xpp::utils::Orientation::RPYRadToQuaternion(curr_ori.p);
  curr.base_.ori.v = curr_ori.v;  //FIXME: these are rpy-vel/acc. transfer to
  curr.base_.ori.a = curr_ori.a;  //       omega vector (world co) -> rpyToEar

  /** 3. Feet Position */
  curr.swingleg_ = false;
  curr.feet_ = nodes_[curr_goal_].feet;

  int sl = nodes_[curr_goal_].swingleg;
  if (sl != NO_SWING_LEG) // only spline foot of swingleg
  {
    double t_upswing = nodes_[curr_goal_].T * kUpswingPercent;

    if ( dt < t_upswing) // leg swinging up
      feet_spliner_up_[sl].GetPoint(dt, curr.feet_[sl]);
    else // leg swinging down
      feet_spliner_down_[sl].GetPoint(dt - t_upswing, curr.feet_[sl]);

    curr.swingleg_[sl] = true;
  }

  LOG4CXX_TRACE(log_, "dt = " << dt << ", curr_goal.T = " << nodes_[curr_goal_].T);

  // to reduce front leg shaking, switch splines one task loop before reaching the end.
  static constexpr double kTaskLoopDuration = 0.005 - 0.0001; // because of rounding errors
  if (dt >= nodes_[curr_goal_].T - kTaskLoopDuration)
    SetCurrGoal(++curr_goal_);

  // sanity check so robot doesn't blow up
  for (LegID leg : LegIDArray)
    if (std::abs(curr.feet_[leg].p(Z)) < 1e-5)
      throw std::logic_error("HyqSpliner::getPoint(): Desired foothold is too close to z=0. "
                             "This is the initial body height.");

  return curr;
}


void HyqSpliner::SetCurrGoal(uint des_goal)
{
  curr_goal_ = des_goal;
  if (curr_goal_ >= nodes_.size())
    throw std::out_of_range("SplinerHolder::NextGoal():\n Goal=" + std::to_string(curr_goal_) + " >= nodes_.size()=" +  std::to_string(nodes_.size()));

  SplineNode from = nodes_.at(curr_goal_-1);
  SplineNode to   = nodes_.at(curr_goal_);

  // Positions, velocities and accelerations
  pos_spliner_.SetBoundary(to.T, from.pos, to.pos);

  // Orientation, angular velocity and angular acceleration
  ori_spliner_.SetBoundary(to.T, from.ori, to.ori);

  // Feet spliner for all legs, even if might be stance legs
  for (LegID leg : LegIDArray) {

    // raise intermediate foothold dependant on foothold difference
    double delta_z = std::abs(to.feet[leg].p(Z) - from.feet[leg].p(Z));
    Point foot_raised = to.feet[leg];
    foot_raised.p[Z] += kLiftHeight + delta_z;

    // move outward only if footholds significantly differ in height
    if (delta_z > 0.01) {
      int sign = (leg == LH || leg == LF) ? 1 : -1;
      foot_raised.p[Y] += sign * kOutwardSwingDistance;
    }

    // upward swing
    double swing_time = to.T;
    feet_spliner_up_[leg].SetBoundary(swing_time, from.feet[leg], foot_raised);

    // downward swing from the foothold at switch state to original
    double t_switch = swing_time * kUpswingPercent;
    Point foot_at_switch;
    feet_spliner_up_[leg].GetPoint(t_switch, foot_at_switch);
    feet_spliner_down_[leg].SetBoundary(swing_time - t_switch,  foot_at_switch, to.feet[leg]);
  }
}


} // namespace hyq
} // namespace xpp
