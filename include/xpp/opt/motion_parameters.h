/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

#include <xpp/utils/endeffectors.h>
#include <map>
#include <memory>

namespace xpp {
namespace opt {

enum MotionTypeID    { WalkID, TrottID, CamelID, BoundID, PushRecID };
enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID };
enum ConstraintName  { InitCom, FinalCom, JunctionCom, Convexity, SuppArea,
                       Dynamic, RomBox, FinalStance, Obstacle };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class MotionParameters {
public:
  using MotionTypePtr   = std::shared_ptr<MotionParameters>;
  using EEID            = xpp::utils::EndeffectorID;
  using Swinglegs       = std::vector<EEID>;
  using SwingLegCycle   = std::vector<Swinglegs>;
  using PosXY           = Eigen::Vector2d;
  using NominalStance   = std::map<EEID, PosXY>;
  using ValXY           = std::array<double,2>;
  using CostWeights     = std::map<CostName, double>;
  using UsedConstraints = std::vector<ConstraintName>;

  virtual ~MotionParameters();

  virtual SwingLegCycle GetOneCycle() const = 0;
  virtual NominalStance GetNominalStanceInBase() const = 0;
  ValXY GetMaximumDeviationFromNominal() const;

  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;

  MotionTypeID id_;
  double t_phase_;
  double max_step_length_;
  double dt_nodes_; ///< time discretization of trajectory for constraints/costs
  int polynomials_per_second_;
  bool start_with_stance_;
  ValXY weight_com_motion_xy_;
  double walking_height_;
  double lift_height_;
  double lambda_deviation_percent_;
  int opt_horizon_in_phases_;


  static MotionTypePtr MakeMotion(MotionTypeID);

protected:
  ValXY max_dev_xy_;

  UsedConstraints constraints_;
  CostWeights cost_weights_;
};

} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
