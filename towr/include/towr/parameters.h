/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_OPTIMIZATION_PARAMETERS_H_
#define TOWR_OPTIMIZATION_PARAMETERS_H_

#include <vector>
#include <array>


namespace towr {

enum ConstraintName { Dynamic, EndeffectorRom, TotalTime, Terrain,
                      Force, Swing, BaseRom, BaseAcc };
enum CostName       { ForcesCostID };

/**
 * @brief Holds the parameters to tune the optimization problem.
 */
struct Parameters {
  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;
  using VecTimes         = std::vector<double>;
  using EEID             = unsigned int;

  friend class NlpFactory;

  /**
   * @brief Default parameters to use.
   */
  Parameters();
  virtual ~Parameters() = default;

  /// Number and initial duration of each foot's swing and stance phases.
  std::vector<VecTimes> ee_phase_durations_;

  /// True if the foot is initially in contact with the terrain.
  std::vector<bool> ee_in_contact_at_start_;

  /// Specifies that timings of all feet, so the gait, should be optimized.
  void OptimizePhaseDurations();

  /**
   * @brief Ensures smooth endeffector motion during swing-phase (recommended)
   */
  void SetSwingConstraint();

  /**
   * Adds base_motion_constraint to restrict 6D base movement. Careful, this
   * can be very limiting.
   *
   * @param dt  interval [s] at which this constraint is enforced.
   */
  void RestrictBaseRangeOfMotion();

  /**
   * @brief Add cost that penalizes large endeffector forces.
   */
  void PenalizeEndeffectorForces();

private:
  /// Which constraints should be used in the optimization problem.
  UsedConstraints constraints_;

  /// Which costs should be used in the optimiation problem.
  CostWeights costs_;

  /// The durations of each base polynomial in the spline (lin+ang).
  VecTimes GetBasePolyDurations() const;

  /// The number of phases allowed for endeffector ee.
  int GetPhaseCount(EEID ee) const;

  /// True if the phase durations should be optimized over.
  bool IsOptimizeTimings() const;

  /// The number of endeffectors.
  int GetEECount() const;

  /// Total duration [s] of the motion.
  double GetTotalTime() const;

  /// Bounds for the phase durations.
  std::array<double,2> GetPhaseDurationBounds() const;

  /// Interval at which the dynamic constraint is enforced.
  double dt_constraint_dynamic_;

  /// Interval at which the range of motion constraint is enforced.
  double dt_constraint_range_of_motion_;

  /// Interval at which the base motion constraint is enforced.
  double dt_constraint_base_motion_;

  /// Fixed duration of each cubic polynomial describing the base motion.
  double duration_base_polynomial_;

  /** Minimum and maximum time for each phase (swing,stance).
   *  Only used when optimizing over phase durations
   */
  std::array<double,2> bound_phase_duration_ = {{0.0, 1e10}};

  /// Number of polynomials to parameterize foot movement during swing phases.
  int ee_polynomials_per_swing_phase_;

  /// Number of polynomials to parameterize each contact force during stance phase.
  int force_polynomials_per_stance_phase_;

  /// The maximum allowable force [N] in normal direction
  double force_limit_in_normal_direction_;

  /**
   * @brief Ensures that the dynamic model is fullfilled at discrete times.
   */
  void SetDynamicConstraint();

  /**
   * @brief Ensures that the range of motion is respected at discrete times.
   */
  void SetKinematicConstraint();

  /**
   * @brief Ensures unilateral forces and inside the friction cone.
   */
  void SetForceConstraint();

  /// which dimensions (x,y,z) of the final base state should be bounded
  std::vector<int> bounds_final_lin_pos,
                   bounds_final_lin_vel,
                   bounds_final_ang_pos,
                   bounds_final_ang_vel;
};

} // namespace towr

#endif /* TOWR_OPTIMIZATION_PARAMETERS_H_ */
