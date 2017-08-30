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
enum ConstraintName  { BasePoly, Dynamic, RomBox, TotalTime, Terrain, Force, Swing };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class OptimizationParameters {
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

  virtual ~OptimizationParameters();


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

  enum BaseRepresentation {CubicHermite, PolyCoeff};
  BaseRepresentation GetBaseRepresentation() const;



  ContactTimings contact_timings_;
  double friction_coeff_ = 1.0; // between terrain and endeffectors


protected:
  ContactSequence contact_sequence_;
  UsedConstraints constraints_;
  CostWeights cost_weights_;
};


// some specific ones
class MonopedOptParameters : public OptimizationParameters {
public:
  MonopedOptParameters();
};

class BipedOptParameters : public OptimizationParameters {
public:
  BipedOptParameters();
};

class QuadrotorOptParameters : public OptimizationParameters {
public:
  QuadrotorOptParameters();
};

class QuadrupedOptParameters : public OptimizationParameters {
public:
  QuadrupedOptParameters();
};






} // namespace opt
} // namespace xpp

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
