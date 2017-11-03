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

#include <xpp_solve/composite.h>

namespace xpp {

enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID, ForcesCostID };
enum ConstraintName  { BasePoly, Dynamic, EndeffectorRom, TotalTime, Terrain,
                       Force, Swing, BaseRom };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class OptimizationParameters {
public:
  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;
  using VecTimes         = std::vector<double>;

  OptimizationParameters();
  virtual ~OptimizationParameters();

  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;

  void SetTotalDuration(double d) {t_total_ = d; };
  double GetTotalTime() const { return t_total_;} ;

  VecTimes GetBasePolyDurations() const;
  bool ConstraintExists(ConstraintName c) const;


  int ee_splines_per_swing_phase_;
  int force_splines_per_stance_phase_;
  int order_coeff_polys_;

  double dt_base_polynomial_;
  double dt_range_of_motion_;
  double dt_base_range_of_motion_;

  double min_phase_duration_;
  double max_phase_duration_;

  enum BaseRepresentation {CubicHermite, PolyCoeff};
  BaseRepresentation GetBaseRepresentation() const;

private:
  double t_total_ = 3.0;
  UsedConstraints constraints_;
  CostWeights cost_weights_;
};

} // namespace xpp

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_MOTION_TYPE_H_ */
