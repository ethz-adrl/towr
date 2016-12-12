/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#ifndef _XPP_XPP_OPT_HYQ_SPLINER_H_
#define _XPP_XPP_OPT_HYQ_SPLINER_H_

#include <xpp/hyq/hyq_state.h>
#include <xpp/utils/polynomial_helpers.h>
#include <xpp/utils/polynomial_xd.h>

#include <xpp/opt/phase.h>
#include <xpp/opt/com_motion.h>
#include <xpp/utils/eigen_std_conversions.h>

namespace xpp {
namespace hyq {

// zmp_ embed this at a smart place
static constexpr int kNee = 4; // number of endeffectors

class SplineNode  {
public:
  using BaseLin3d    = xpp::utils::StateLin3d;
  using BaseAng3d    = xpp::utils::StateAng3d;
  using BaseLin1d    = xpp::utils::StateLin1d;
  using Vector3d     = Eigen::Vector3d;
  using FeetArray    = std::array<BaseLin3d, kNee>;
  using ContactArray = std::array<bool, kNee>;

  SplineNode(){};
  SplineNode(const HyqState& state_joints, double t_max);

  FeetArray feet_W_;
  ContactArray swingleg_;

//  LegDataMap<BaseLin3d> feet_W_; ///< contacts expressed in world frame
//  LegDataMap< bool > swingleg_;
  BaseAng3d base_ang_;
  BaseLin1d base_z_;

  double T;                      ///< time to reach this state

  std::array<Vector3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;
};

/** @brief Splines the base pose (only z-position + orientation).
  */
class HyqSpliner {
public:
  using ComMotionS     = std::shared_ptr<xpp::opt::ComMotion>;
  using Vector3d      = Eigen::Vector3d;
  using VecFoothold   = utils::StdVecEigen2d;
  using State3d       = xpp::utils::StateLin3d;
  using HyqStateVec   = std::vector<HyqState>;
  // mpc don't forget about the spliner order
  using SplinerOri    = xpp::utils::PolynomialXd< utils::CubicPolynomial, State3d>;
  using SplinerFeet   = xpp::utils::PolynomialXd< utils::QuinticPolynomial, State3d>;
  using ZPolynomial   = xpp::utils::CubicPolynomial;

  using FeetArray     = SplineNode::FeetArray;
  using ContactArray  = SplineNode::ContactArray;

public:
  HyqSpliner();
  virtual ~HyqSpliner();

  void SetParams(double upswing, double lift_height,
                 double outward_swing_distance,
                 double discretization_time);

  void Init(const xpp::opt::PhaseVec&,
            const ComMotionS&,
            const VecFoothold&,
            double des_height,
            const HyqState& curr_state);

  HyqStateVec BuildWholeBodyTrajectoryJoints() const;

private:
  std::vector<SplineNode> nodes_; // the discrete states to spline through
  std::vector<ZPolynomial> z_spliner_;
  std::vector<SplinerOri> ori_spliner_;
  std::vector<LegDataMap< SplinerFeet > > feet_spliner_up_, feet_spliner_down_;
  ComMotionS com_motion_;

  double kDiscretizationTime;   // at what interval the continuous trajectory is sampled
  double kUpswingPercent;       // how long to swing up during swing
  double kLiftHeight;           // how high to lift the leg
  double kOutwardSwingDistance; // how far to swing leg outward (y-dir)

  std::vector<SplineNode> BuildNodeSequence(const HyqState& P_init,
                                            const xpp::opt::PhaseVec&,
                                            const VecFoothold& footholds,
                                            double des_robot_height);

  void CreateAllSplines(const std::vector<SplineNode>& nodes);

//  SplineNodeVec GetInterpolatedNodes() const;
  State3d GetCurrPosition(double t_global) const;
  xpp::utils::StateAng3d GetCurrOrientation(double t_global) const;
  void FillCurrFeet(double t_global, FeetArray& feet, ContactArray& swingleg) const;
  void FillZState(double t_global, State3d& pos) const;

//  Spliner3d BuildPositionSpline(const SplineNode& from, const SplineNode& to) const;
  SplinerOri BuildOrientationRpySpline(const SplineNode& from, const SplineNode& to) const;
  LegDataMap<SplinerFeet> BuildFootstepSplineUp(const SplineNode& from, const SplineNode& to) const;
  LegDataMap<SplinerFeet> BuildFootstepSplineDown(const FeetArray& feet_at_switch,const SplineNode& to) const;

  void BuildOneSegment(const SplineNode& from, const SplineNode& to,
                       ZPolynomial& z_poly,
                       SplinerOri& ori,
                       LegDataMap< SplinerFeet >& feet_up,
                       LegDataMap< SplinerFeet >& feet_down) const;

  static Vector3d TransformQuatToRpy(const Eigen::Quaterniond& q);
  int GetSplineID(double t_global) const;
  double GetLocalSplineTime(double t_global) const;
  double GetTotalTime() const;
};

} // namespace hyq
} // namespace xpp

#endif // _XPP_XPP_OPT_HYQ_SPLINER_H_
