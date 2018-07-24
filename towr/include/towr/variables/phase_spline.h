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

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_PHASE_SPLINE_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_PHASE_SPLINE_H_

#include "node_spline.h"
#include "phase_durations_observer.h"
#include "nodes_variables_phase_based.h"

namespace towr {

/**
 * @brief A spline built from node values and polynomial durations.
 *
 * This class is responsible for combining the optimized node values with
 * the optimized phase durations to construct a sequence of
 * CubicHermitePolynomial. For this it observers whether one of the quantities
 * changed and then updates all the polynomials accordingly.
 */
class PhaseSpline : public NodeSpline, public PhaseDurationsObserver{
public:
  using Ptr = std::shared_ptr<PhaseSpline>;
  using VectorXd = Eigen::VectorXd;

  /**
   * @brief Constructs a spline with varying/optimized phase durations.
   * @param node_variables The optimized node variables (pos, vel).
   * @param phase_durations Pointer to the changing phase duration variables.
   */
  PhaseSpline(NodesVariablesPhaseBased::Ptr const node_variables,
              PhaseDurations* const phase_durations);
  ~PhaseSpline() = default;

  /**
   * @brief Called by subject to update polynomial durations when they changed.
   */
  void UpdatePolynomialDurations() override;

  /**
   * @brief How the spline position changes when the polynomial durations change.
   * @param t  The time along the spline at which the sensitivity is required.
   * @return the pxn Jacobian, where:
   *             p: Number of dimensions of the spline
   *             n: Number of optimized durations.
   */
  Jacobian GetJacobianOfPosWrtDurations(double t) const override;

private:
  /**
   * @brief How the position at time t changes with current phase duration.
   * @param t The global time along the spline.
   * @return How a duration change affects the x,y,z position.
   */
  Eigen::VectorXd GetDerivativeOfPosWrtPhaseDuration (double t) const;

  NodesVariablesPhaseBased::Ptr phase_nodes_; // retain pointer for extended functionality
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_PHASE_SPLINE_H_ */
