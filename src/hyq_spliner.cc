/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/hyq/hyq_spliner.h>

#include <kindr/rotations/RotationEigen.hpp>

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
               double outward_swing_distance)
{
  kUpswingPercent = upswing;
  kLiftHeight = lift_height;
  kOutwardSwingDistance = outward_swing_distance;

}


std::vector<SplineNode>
HyqSpliner::BuildStepSequence(const HyqState& P_init,
                              const VecZmpSpline& zmp_splines,
                              const VecFoothold& footholds,
                              double robot_height)
{
  std::vector<SplineNode> nodes;
  using namespace xpp::utils::coords_wrapper; // pulls in X, Y, Z..


  // start node
  nodes.push_back(BuildNode(P_init, std::numeric_limits<double>::infinity()));

  /** Add state sequence based on footsteps **/
  // desired state after first 4 leg support phase
  HyqState P_plan_prev = P_init;
  P_plan_prev.ZeroVelAcc();
  for (hyq::LegID l : hyq::LegIDArray) {
    P_plan_prev.feet_[l].p(Z) = 0.0;
  }
  P_plan_prev.base_.pos.p(Z) = robot_height + P_plan_prev.GetZAvg(); // height of footholds

  for (const ZmpSpline& s : zmp_splines)
  {
    // copy a few values from previous state
    HyqState P_plan = P_plan_prev;
    P_plan.swingleg_ = false;

    // only change state in swingphase
    if (!s.IsFourLegSupport()) {
      const Foothold& f = footholds.at(s.GetCurrStep());
      P_plan.swingleg_[f.leg] = true;
      P_plan.feet_[f.leg].p(X) = f.p(X);
      P_plan.feet_[f.leg].p(Y) = f.p(Y);
      P_plan.feet_[f.leg].p(Z) = f.p(Z);

      // adjust orientation depending on footholds
      std::array<Vector3d, kNumSides> avg = P_plan.GetAvgSides();
      // fixme calculate distance based on current foothold position, not fixed values
      double width_hip = 0.414;
      double length_hip = 0.747;
      double i_roll  = std::atan2((avg[LEFT_SIDE](Z) - avg[RIGHT_SIDE](Z)), width_hip);
      double i_pitch = std::atan2((avg[HIND_SIDE](Z) - avg[FRONT_SIDE](Z)), length_hip);


      // how strictly the body should follow difference in foothold height
      double roll_gain = 0.0;
      double pitch_gain = 0.0;
      // Quaternion is the same for angle += i*pi
      kindr::rotations::eigen_impl::EulerAnglesXyzPD yprIB(
          roll_gain*i_roll,
          pitch_gain*i_pitch,
          0.0); // P_yaw_des.at(s.step_));


      kindr::rotations::eigen_impl::RotationQuaternionPD qIB(yprIB);
      P_plan.base_.ori.q = qIB.toImplementation();

      // adjust global z position of body depending on footholds
      P_plan.base_.pos.p(Z) = robot_height + P_plan.GetZAvg(); // height of footholds
    }


    nodes.push_back(BuildNode(P_plan, s.GetDuration()));
    P_plan_prev = P_plan;
  }


  return nodes;
}


SplineNode
HyqSpliner::BuildNode(const HyqState& state, double t_max)
{
  kindr::rotations::eigen_impl::RotationQuaternionPD qIB(state.base_.ori.q);

  kindr::rotations::eigen_impl::EulerAnglesXyzPD rpyIB(qIB);
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
  kindr::rotations::eigen_impl::EulerAnglesXyzPD yprIB_full = rpyIB;
  yprIB_full.setYaw(rpyIB.yaw() + counter360*2*M_PI);
  Point ori = Point(yprIB_full.toImplementation());


  return SplineNode(state.base_.pos, ori, state.feet_, state.SwinglegID(), t_max);
}


HyqState HyqSpliner::getPoint(double t_global)
{

  HyqState curr;

  /** Transform global time to local spline time dt */
  double dt = t_global;
  for (uint n = 1; n < curr_goal_; n++) {
    dt -= nodes_[n].T;
  }

  /** Positions */
  pos_spliner_.GetPoint(dt, curr.base_.pos);

  /** Orientations */
  Point curr_ori, curr_foot;
  ori_spliner_.GetPoint(dt, curr_ori);

  kindr::rotations::eigen_impl::EulerAnglesXyzPD yprIB(curr_ori.p);
  kindr::rotations::eigen_impl::RotationQuaternionPD qIB(yprIB);
  curr.base_.ori.q = qIB.toImplementation();
  curr.base_.ori.v = curr_ori.v;
  curr.base_.ori.a = curr_ori.a;

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


  if (dt >= nodes_[curr_goal_].T)
    SetCurrGoal(++curr_goal_);

  // sanity check so robot doesn't blow up
  for (LegID leg : LegIDArray)
    if (std::abs(curr.feet_[leg].p(Z)) > 0.5)
      throw std::logic_error("HyqSpliner::getPoint(): Desired foothold is too high. "
                             "Ground plane is at zero.");

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
