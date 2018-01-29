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


namespace towr {

enum ConstraintName { Dynamic, EndeffectorRom, TotalTime, Terrain,
                       Force, Swing, BaseRom };
enum CostName       { ForcesCostID };

/**
 * @brief Holds the parameters to tune the optimization problem.
 */
struct Parameters {

  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;
  using VecTimes         = std::vector<double>;
  using EEID             = unsigned int;

  /**
   * @brief Default parameters to use.
   */
  Parameters();
  virtual ~Parameters() = default;

  /// Which constraints should be used in the optimization problem.
  UsedConstraints constraints_;

  /// Which costs should be used in the optimiation problem.
  CostWeights costs_;

  /// Total duration [s] of the walking motion.
  double t_total_;

  /// Interval at which the range of motion constraint is enforced.
  double dt_constraint_range_of_motion_;

  /// Interval at which the base motion constraint is enforced.
  double dt_constraint_base_motion_;

  /// Fixed duration of each cubic polynomial describing the base motion.
  double duration_base_polynomial_;

  /// Number and initial duration of each foot's swing and stance phases.
  std::vector<VecTimes> ee_phase_durations_;

  /// True if the foot is initially in contact with the terrain.
  std::vector<bool> ee_in_contact_at_start_;

  /// When optimizing over phase duration, this is the minimum allowed.
  double min_phase_duration_;

  /// When optimizing over phase duration, this is is maximum allowed.
  double max_phase_duration_;

  /// Number of polynomials to parameterize foot movement during swing phases.
  int ee_polynomials_per_swing_phase_;

  /// Number of polynomials to parameterize each contact force during stance phase.
  int force_polynomials_per_stance_phase_;

  /// The maximum allowable force [N] in normal direction
  double force_limit_in_norm_;



  /// The durations of each base polynomial in the spline (lin+ang).
  VecTimes GetBasePolyDurations() const;

  /// The number of phases allowed for endeffector ee.
  int GetPhaseCount(EEID ee) const;

  /// True if the phase durations should be optimized over.
  bool OptimizeTimings() const;

  /// The number of endeffectors.
  int GetEECount() const;
};

} // namespace towr

#endif /* TOWR_OPTIMIZATION_PARAMETERS_H_ */
