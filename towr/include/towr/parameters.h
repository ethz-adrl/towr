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
#include <utility> // std::pair, std::make_pair

namespace towr {

/**
 * @defgroup Parameters
 * @brief %Parameters to tune the optimization problem.
 *
 * The number of parameters to tune is relatively small (~10), however, they
 * do have a large impact on speed and convergence of the optimizer.
 */

/**
 * @brief The parameters to tune the optimization problem.
 *
 * The number of parameters to tune is relatively small (~10), however, they
 * do have a large impact on speed and convergence of the optimizer. Below
 * we give some insights into how different values affect the solution and
 * what should be taken into consideration when tuning these. For more
 * background knowledge, refer to the corresponding
 * [paper](https://doi.org/10.1109/LRA.2018.2798285).
 *
 * #### Towr problem formulation
 * \image html towr_problem_with_code.png
 *
 * ### Constraint discretization ###
 * A factor that strongly impacts the solution time is how
 * often the DynamicConstraint and the RangeOfMotionConstraint are enforced
 * along the trajectory (given by the values of @ref dt_constraint_dynamic_
 * and @ref dt_constraint_range_of_motion_). Increasing the discretization
 * interval of e.g. @ref dt_constraint_range_of_motion_ to 1.5s greatly speeds
 * up the process. However, if the discretization is too coarse, the motion
 * of e.g. the endeffector can move way outside the bounding boxes, as long it
 * comes back inside this box at the few times the constraint is actually
 * enforced. This is a valid solution to the optimization problem, but of
 * course physically infeasible. Therefore, when the solution shows the
 * feet shooting quickly through 3D space, @ref dt_constraint_range_of_motion_
 * should probably be decreased. The same logic holds for too large
 * @ref dt_constraint_dynamic_: This can produce motion that violate the
 * dynamics at most times, except exactly where they are enforced. This leads
 * to unphysical motions that will be impossible to track on a real system. The
 * more finely the constraint discretization is set, the more sure one can be
 * that the dynamic model is being respected.
 *
 * ### Number of optimization variables ###
 * In order to shorten the solution time, another way is to use less
 * polynomials, but each of longer duration. This can be achieved by increasing the
 * @ref duration_base_polynomial_. However, the longer this duration becomes, the
 * less parameters (freedom), the solver has to find a solution that
 * fullfills all the constraints, so it becomes more likely that no solution
 * is found. This variable is related to @ref dt_constraint_dynamic_ --
 * if the dynamic constraint should be enforced at very short intervals,
 * it is often also required that the base motion has enough freedom (short
 * durations) to enable this.
 *
 * ### Available steps ###
 * One of the main reasons that the solver **fails to find a solution**, is when
 * there are are not enough steps available to reach a goal position that is
 * either too far away, or the terrain too complex. The number of steps per leg
 * and their initial durations are set by @ref ee_phase_durations_. On the other
 * hand, too many steps per leg are hardly an issue and the solver simply
 * generates more short steps, but finds a solution nonetheless. Therefore:
 *   * push back more phases for each foot into this vector to give the
 *     optimizer more freedom in finding a solution.
 *
 * ### Jittering (and cost terms) ###
 * Sometimes the solution to the optimization problem looks jerky, with the
 * forces and base motion quickly jittering back and forth. This is because
 * by default, we don't include any @ref costs_ in the formulation. The solver
 * has too many optimization variables (degrees of freedom) to modify and only
 * few constraints that restrict the motion, so these extra and unnecessary
 * values can be set to extreme values. The cleanest way to counter this
 * is to add a cost term that e.g. penalizes base and endeffector accelerations.
 * See @ref Costs for some inspiration. We try to avoid cost terms if possible,
 * as they require tuning different weighing parameters w.r.t. each other and
 * make the problem slower. Other ways to remove the jittering are as follows:
 *   * Increase @ref duration_base_polynomial_ to remove some DoF.
 *   * Decrease the overall time-horizon of the optimization problem. This is
 *     determined by the sum of all elements in @ref ee_phase_durations_. A
 *     shorter time horizon means less base polynomials, therefore less
 *     extra degrees of freedom for the optimizer to set to extreme values.
 *   * Decrease @ref force_polynomials_per_stance_phase_ to have less jittering
 *     force profiles and therefore also less jittering base-motions.
 *
 * ### Optimizing the gait ###
 * The solver is able to modify the gait to best achieve a given task. This
 * can be turned on by calling OptimizePhaseDurations(). This can help the
 * solver find a solution to terrains which cannot be crossed with the
 * initialized gait. However, this optimization over the phase
 * durations, which affect the entire motion of that endeffector, reduce the
 * sparsity of the formulation. This increases the chances of getting stuck
 * in local minima, as well as increases the computation time. Therefore,
 * this option should be treated with caution. An alternative to turning on
 * this option is initializing with different gaits and/or changing the
 * parameters described above.
 *
 * @ingroup Parameters
 */
class Parameters {
public:
  /**
   * @brief Identifiers to be used to add certain constraints to the
   * optimization problem.
   */
  enum ConstraintName { Dynamic,        ///< sets DynamicConstraint
                        EndeffectorRom, ///< sets RangeOfMotionConstraint
                        TotalTime,      ///< sets TotalDurationConstraint
                        Terrain,        ///< sets TerrainConstraint
                        Force,          ///< sets ForceConstraint
                        Swing,          ///< sets SwingConstraint
                        BaseRom,        ///< sets BaseMotionConstraint
                        BaseAcc         ///< sets SplineAccConstraint
  };
  /**
   *  @brief Indentifiers to be used to add certain costs to the optimization
   *  problem.
   */
  enum CostName       { ForcesCostID,    ///< sets NodeCost on force nodes
                        EEMotionCostID   ///< sets NodeCost on endeffector velocity
  };

  using CostWeights      = std::vector<std::pair<CostName, double>>;
  using UsedConstraints  = std::vector<ConstraintName>;
  using VecTimes         = std::vector<double>;
  using EEID             = unsigned int;

  /**
   * @brief Default parameters to get started.
   */
  Parameters();
  virtual ~Parameters() = default;

  /// Number and initial duration of each foot's swing and stance phases.
  std::vector<VecTimes> ee_phase_durations_;

  /// True if the foot is initially in contact with the terrain.
  std::vector<bool> ee_in_contact_at_start_;

  /// Which constraints should be used in the optimization problem.
  UsedConstraints constraints_;

  /// Which costs should be used in the optimiation problem.
  CostWeights costs_;

  /// Interval at which the dynamic constraint is enforced.
  double dt_constraint_dynamic_;

  /// Interval at which the range of motion constraint is enforced.
  double dt_constraint_range_of_motion_;

  /// Interval at which the base motion constraint is enforced.
  double dt_constraint_base_motion_;

  /// Fixed duration of each cubic polynomial describing the base motion.
  double duration_base_polynomial_;

  /// Number of polynomials to parameterize foot movement during swing phases.
  int ee_polynomials_per_swing_phase_;

  /// Number of polynomials to parameterize each contact force during stance phase.
  int force_polynomials_per_stance_phase_;

  /// The maximum allowable force [N] in normal direction
  double force_limit_in_normal_direction_;

  /// which dimensions (x,y,z) of the final base state should be bounded
  std::vector<int> bounds_final_lin_pos_,
                   bounds_final_lin_vel_,
                   bounds_final_ang_pos_,
                   bounds_final_ang_vel_;

  /** Minimum and maximum time [s] for each phase (swing,stance).
   *
   *  Only used when optimizing over phase durations.
   *  Make sure max time is less than total duration of trajectory, or segfault.
   *  limiting this range can help convergence when optimizing gait.
   */
  std::pair<double,double> bound_phase_duration_;

  /// Specifies that timings of all feet, so the gait, should be optimized.
  void OptimizePhaseDurations();

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
};

} // namespace towr

#endif /* TOWR_OPTIMIZATION_PARAMETERS_H_ */
