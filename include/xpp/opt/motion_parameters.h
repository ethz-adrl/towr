/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

#include <xpp/endeffectors.h>
#include <map>
#include <memory>

namespace xpp {
namespace opt {

enum MotionTypeID    { WalkID, TrotID, PaceID, BoundID, PushRecID };
enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID };
enum ConstraintName  { InitCom, FinalCom, JunctionCom, Convexity,
                       Dynamic, RomBox, Stance, Obstacle };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class MotionParameters {
public:
  using MotionTypePtr    = std::shared_ptr<MotionParameters>;
  using EEID             = EndeffectorID;
  using EEVec            = std::vector<EEID>;
  using EECycleVec2      = std::vector<EndeffectorsBool>;
  using PhaseTimings     = std::vector<double>;
  using Phase            = std::pair<EndeffectorsBool, double>;
  using PhaseVec         = std::vector<Phase>;


  using PosXY            = Eigen::Vector2d;
  using PosXYZ           = Eigen::Vector3d;
  using NominalStance    = EndeffectorsPos;
  using ValXY            = std::array<double,2>;
  using CostWeights      = std::map<CostName, double>;
  using UsedConstraints  = std::vector<ConstraintName>;

  virtual ~MotionParameters();

  int GetEECount() const { return robot_ee_.size(); };

  NominalStance GetNominalStanceInBase() const { return nominal_stance_; };
  PhaseVec GetOneCycle() const;

  ValXY GetMaximumDeviationFromNominal() const;

  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;

  MotionTypeID id_;
  double max_step_length_;
  double dt_nodes_; ///< time discretization of trajectory for constraints/costs
  int polynomials_per_second_;
  ValXY weight_com_motion_xy_;
  double geom_walking_height_;
  double lift_height_;
  int opt_horizon_in_phases_;
  PosXYZ offset_geom_to_com_; ///< between CoM and geometric center

  NominalStance nominal_stance_;



  EECycleVec2 ee_cycle2_;

  PhaseTimings timings_;
  EEVec robot_ee_;





protected:
  ValXY max_dev_xy_;

  UsedConstraints constraints_;
  CostWeights cost_weights_;
};

} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
