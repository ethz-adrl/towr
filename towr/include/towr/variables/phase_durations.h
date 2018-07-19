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

#ifndef TOWR_VARIABLES_CONTACT_SCHEDULE_H_
#define TOWR_VARIABLES_CONTACT_SCHEDULE_H_

#include <ifopt/variable_set.h>

#include "phase_durations_observer.h"

namespace towr {

/**
 * @brief A variable set composed of the phase durations of an endeffector.
 *
 * This class holds the current variables determining the alternating swing-
 * and stance durations of one foot. These durations are then used in the
 * Spline together with NodeVariables to construct foot motions and forces.
 *
 * See this explanation: https://youtu.be/KhWuLvb934g?t=788
 *
 * @ingroup Variables
 */
class PhaseDurations : public ifopt::VariableSet {
public:
  using Ptr           = std::shared_ptr<PhaseDurations>;
  using VecDurations  = std::vector<double>;
  using EndeffectorID = uint;


  /**
   * @brief Constructs a variable set for a specific endeffector
   * @param ee  The endeffector ID which these durations apply to.
   * @param initial_durations  Initial values for the optimization variables.
   * @param min_phase_duration  The minimum allowable time for one phase.
   * @param max_phase_duration  The maximum allowable time for one phase.
   */
  PhaseDurations (EndeffectorID ee,
                  const VecDurations& initial_durations,
                  bool is_first_phase_in_contact,
                  double min_phase_duration,
                  double max_phase_duration);
  virtual ~PhaseDurations () = default;

  /**
   * @returns The durations (stance, swing, ...) for each phase of this foot.
   */
  VecDurations GetPhaseDurations() const;

  /**
   * @returns The optimization variables (phase durations [s]).
   */
  VectorXd GetValues() const override;

  /**
   * @brief  Sets the phase durations from pure Eigen optimization variables.
   */
  void SetVariables(const VectorXd& x) override;

  /**
   * @returns The maximum and minimum time each phase is allowed to take.
   */
  VecBound GetBounds () const override;

  /**
   * @brief How a change in the phase durations affect the position of a spline.
   * @param phase  The current phase the spline is in at time t.
   * @param dx_dT  The derivative of the spline position w.r.t the phase durations T.
   * @param xd     The velocity of the spline at the current time t.
   * @return A pxn dimensional Jacobian where:
   *           p: number of dimensions of the spline (e.g. x,y,z)
   *           n: number of phase durations in this class that are optimized over.
   *
   * This method is related to constructing a cubic-Hermite spline with
   * these phase durations. Leaving the node values unchanged, the shape of the
   * spline can also be changed by expanding or compressing the individual
   * polynomials through the duration. This method quantifies the sensitivity
   * of the spline position on these durations.
   */
  Jacobian GetJacobianOfPos(int phase, const VectorXd& dx_dT, const VectorXd& xd) const;

  /**
   * @brief Adds observer that is updated every time new variables are set.
   * @param spline  A pointer to a Hermite spline using the durations.
   */
  void AddObserver(PhaseDurationsObserver* const spline);

  /**
   * @brief Whether the endeffector is in contact with the environment.
   * @param t  global time along the trajectory.
   */
  bool IsContactPhase(double t) const;

private:
  VecDurations durations_;

  double t_total_;
  bool initial_contact_state_; ///< true if first phase in contact
  ifopt::Bounds phase_duration_bounds_;

  std::vector<PhaseDurationsObserver*> observers_;
  void UpdateObservers() const;
};

} /* namespace towr */

#endif /* TOWR_VARIABLES_CONTACT_SCHEDULE_H_ */
