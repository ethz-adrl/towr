/**
@file    motion_type.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Jan 11, 2017
@brief   Brief description
 */

#ifndef TOWR_OPTIMIZATION_PARAMETERS_H_
#define TOWR_OPTIMIZATION_PARAMETERS_H_


#include <utility>
#include <vector>


namespace towr {


enum ConstraintName  { Dynamic, EndeffectorRom, TotalTime, Terrain,
                       Force, Swing, BaseRom };
enum CostName        { ComCostID, RangOfMotionCostID, PolyCenterCostID,
                       FinalComCostID, FinalStanceCostID, ForcesCostID };

/** This class holds all the hardcoded values describing a motion.
  * This is specific to the robot and the type of motion desired.
  */
class OptimizationParameters {
public:
  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;
  using VecTimes         = std::vector<double>;
  using EEID             = unsigned int;

  OptimizationParameters();
  virtual ~OptimizationParameters() = default;

  UsedConstraints GetUsedConstraints() const;
  CostWeights GetCostWeights() const;


  void SetPhaseDurations(const std::vector<VecTimes>& phase_durations,
                         const std::vector<bool>& initial_contact);



  void SetTotalDuration(double d) {t_total_ = d; };
  double GetTotalTime() const { return t_total_;} ;

  VecTimes GetBasePolyDurations() const;
  VecTimes GetEEPhaseDurations(EEID) const;
  bool IsEEInContactAtStart(EEID) const;
  int GetPhaseCount(EEID) const;

  bool OptimizeTimings() const;
  int GetEECount() const  { return ee_in_contact_at_start_.size(); };

  double GetForceLimit() const { return force_z_limit_; };


  int ee_splines_per_swing_phase_;
  int force_splines_per_stance_phase_;

  double dt_base_polynomial_;
  double dt_range_of_motion_;
  double dt_base_range_of_motion_;

  double min_phase_duration_;
  double max_phase_duration_;

private:
  double t_total_ = 3.0;
  UsedConstraints constraints_;
  CostWeights cost_weights_;

  std::vector<VecTimes> ee_phase_durations_; // smell make private again
  std::vector<bool> ee_in_contact_at_start_; // smell make private again

  double force_z_limit_; // limit of force in normal direction

};

} // namespace towr

#endif /* TOWR_OPTIMIZATION_PARAMETERS_H_ */
