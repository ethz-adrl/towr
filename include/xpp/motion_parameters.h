/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_

#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include <xpp/composite.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

namespace xpp {
namespace opt {

enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID, ForcesCostID };
enum ConstraintName  { State, JunctionCom, Dynamic, RomBox, TotalTime, Terrain };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class MotionParameters {
public:
  using EEID             = EndeffectorID;
  using EEVec            = std::vector<EEID>;
  using ContactSequence  = std::vector<EndeffectorsBool>;
  using ContactTimings   = std::vector<std::vector<double>>;
  using Phase            = std::pair<EndeffectorsBool, double>;
  using ContactSchedule  = std::vector<Phase>;


  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;

  using VecTimes         = std::vector<double>;

  virtual ~MotionParameters();


  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;
  double GetTotalTime() const;

  VecTimes GetBasePolyDurations() const;
  bool ConstraintExists(ConstraintName c) const;


  int ee_splines_per_swing_phase_;
  int force_splines_per_stance_phase_;
  int order_coeff_polys_;

  double dt_base_polynomial_;
  double dt_range_of_motion_;

  double min_phase_duration_;
  double max_phase_duration_;


  double GetForceLimit() const {return force_limit_; };

  ContactTimings contact_timings_;


protected:
  double force_limit_;


  ContactSequence contact_sequence_;
  UsedConstraints constraints_;
  CostWeights cost_weights_;
};

} // namespace opt
} // namespace hyq

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
