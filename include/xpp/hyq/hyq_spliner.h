/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#ifndef _XPP_HYQ_SPLINER_H_
#define _XPP_HYQ_SPLINER_H_

#include <xpp/hyq/hyq_state.h>
#include <xpp/utils/polynomial_helpers.h>
#include <xpp/utils/polynomial_xd.h>
#include <xpp/opt/phase_info.h>

namespace xpp {
namespace hyq {

class SplineNode : public HyqState {
public:
  using BaseLin3d = xpp::utils::BaseLin3d;
  using Vector3d  = Eigen::Vector3d;

  SplineNode(){};
  SplineNode(const HyqStateJoints& state_joints, double t_max);

  LegDataMap<BaseLin3d> feet_W_; ///< contacts expressed in world frame
  double T;                      ///< time to reach this state

  std::array<Vector3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;
};

/** @brief Splines the base pose (only z-position + orientation).
  */
class HyqSpliner {
public:
  using ComSpline = std::vector<xpp::utils::ComPolynomial>;
  using Vector3d = Eigen::Vector3d;
  using VecFoothold = Foothold::VecFoothold;
  using Point         = ::xpp::utils::BaseLin3d;
  using SplineNodeVec = std::vector<SplineNode>;
  using HyqStateVec   = std::vector<HyqStateJoints>;
  using Spliner3d = ::xpp::utils::PolynomialXd< ::xpp::utils::QuinticPolynomial,::xpp::utils::kDim3d, ::xpp::utils::BaseLin3d>;

public:
  HyqSpliner();
  virtual ~HyqSpliner();

  void SetParams(double upswing, double lift_height,
                 double outward_swing_distance,
                 double discretization_time);

  void Init(const xpp::opt::PhaseVec&,
            const ComSpline&,
            const VecFoothold&,
            double des_height,
            const HyqStateJoints& curr_state);

  HyqStateVec BuildWholeBodyTrajectoryJoints() const;

private:
  std::vector<SplineNode> nodes_; // the discrete states to spline through
  std::vector<Spliner3d> pos_spliner_, ori_spliner_;
  std::vector<LegDataMap< Spliner3d > > feet_spliner_up_, feet_spliner_down_;
  ComSpline optimized_xy_spline_;

  double kDiscretizationTime;   // at what interval the continuous trajectory is sampled
  double kUpswingPercent;       // how long to swing up during swing
  double kLiftHeight;           // how high to lift the leg
  double kOutwardSwingDistance; // how far to swing leg outward (y-dir)

  std::vector<SplineNode> BuildNodeSequence(const HyqStateJoints& P_init,
                                            const xpp::opt::PhaseVec&,
                                            const VecFoothold& footholds,
                                            double des_robot_height);

  void CreateAllSplines(const std::vector<SplineNode>& nodes);

  SplineNodeVec GetInterpolatedNodes() const;
  Point GetCurrPosition(double t_global) const;
  xpp::utils::BaseAng3d GetCurrOrientation(double t_global) const;
  void FillCurrFeet(double t_global, LegDataMap<Point>& feet, LegDataMap<bool>& swingleg) const;
  void FillZState(double t_global, Point& pos) const;

  Spliner3d BuildPositionSpline(const SplineNode& from, const SplineNode& to) const;
  Spliner3d BuildOrientationRpySpline(const SplineNode& from, const SplineNode& to) const;
  LegDataMap<Spliner3d> BuildFootstepSplineUp(const SplineNode& from, const SplineNode& to) const;
  LegDataMap<Spliner3d> BuildFootstepSplineDown(const LegDataMap<Point>& feet_at_switch,const SplineNode& to) const;

  void BuildOneSegment(const SplineNode& from, const SplineNode& to,
                       Spliner3d& pos, Spliner3d& ori,
                       LegDataMap< Spliner3d >& feet_up,
                       LegDataMap< Spliner3d >& feet_down) const;

  static Vector3d TransformQuatToRpy(const Eigen::Quaterniond& q);
  int GetSplineID(double t_global) const;
  double GetLocalSplineTime(double t_global) const;
  double GetTotalTime() const;
};

} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_SPLINER_H_
