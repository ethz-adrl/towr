/**
@file    wb_traj_generator.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2016
@brief   Defines the class WholeBody Trajectory Generator
 */

#ifndef _XPP_XPP_OPT_WB_TRAJ_GENERATOR_H_
#define _XPP_XPP_OPT_WB_TRAJ_GENERATOR_H_

#include "com_motion.h"
#include "motion_phase.h"
#include "polynomial_xd.h"
#include "com_polynomial_helpers.h"
#include "endeffectors_motion.h"

namespace xpp {
namespace opt {


/** @brief Whole-Body Trajectory Generator
  *
  * This class is responsible for taking the optimized trajectory and
  * filling in the remaining DoF to produce a discretized whole body trajectory.
  * The DoF that are calculated by this class include:
  *   - Body height
  *   - Angular pos/vel/acc
  *   - Swingleg trajectories.
  */
class WBTrajGenerator {
public:
  using ComMotionS    = std::shared_ptr<xpp::opt::ComMotion>;
  using Vector3d      = Eigen::Vector3d;
  using VecFoothold   = std::vector<Contact>;
  using SplinerOri    = PolynomialXd< CubicPolynomial, StateLin3d>;
  using ZPolynomial   = LinearPolynomial;
  using PhaseVec      = std::vector<MotionPhase>;

  using SplineNode     = RobotStateCartesian;
  using FeetArray      = typename SplineNode::FeetArray;
  using ContactArray   = typename SplineNode::ContactState;
  using ArtiRobVec     = std::vector<SplineNode>;
  using EEID           = EndeffectorID;
  using EEPtr          = std::shared_ptr<EndeffectorsMotion>;

public:
  WBTrajGenerator();
  virtual ~WBTrajGenerator();

  void Init(const PhaseVec&,
            const ComMotionS&,
            const VecFoothold&,
            const EEPtr&,
            const SplineNode& curr_state,
            double lift_height,
            const Vector3d& com_offset);

  ArtiRobVec BuildWholeBodyTrajectory(double dt) const;

private:
  int kNEE;
  double t_start_;
  int phase_start_;
  Vector3d offset_geom_to_com_; ///< difference between com and geometric center

  std::vector<SplineNode> nodes_;
  std::vector<ZPolynomial> z_spliner_;
  std::vector<SplinerOri> ori_spliner_;
  EEPtr ee_spliner_;
  ComMotionS com_motion_;

  double leg_lift_height_;  ///< how high to lift the leg

  void BuildNodeSequence(const PhaseVec&, const VecFoothold& footholds);

  void CreateAllSplines();

  StateLin3d GetCurrPosition(double t_global) const;
  StateAng3d GetCurrOrientation(double t_global) const;
  State3d GetCurrentBase(double t_global) const;
  ContactArray GetCurrContactState(double t_gloal) const;

  void FillZState(double t_global, StateLin3d& pos) const;

  void BuildPhase(const SplineNode& from, const SplineNode& to,
                       ZPolynomial& z_poly,
                       SplinerOri& ori) const;

  static Vector3d TransformQuatToRpy(const Eigen::Quaterniond& q);
  int GetPhaseID(double t_global) const;
  double GetLocalPhaseTime(double t_global) const;
  double GetTotalTime() const;
  double GetPercentOfPhase(double t_global) const;

  SplineNode GetRobotState(double t_global) const;
};

} // namespace opt
} // namespace xpp

#include "impl/wb_traj_generator-impl.h"

#endif // _XPP_XPP_OPT_WB_TRAJ_GENERATOR_H_
