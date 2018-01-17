/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of ETH ZURICH nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef TOWR_VARIABLES_SPLINE_H_
#define TOWR_VARIABLES_SPLINE_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Sparse>

#include "polynomial.h"
#include "nodes_observer.h"
#include "phase_durations_observer.h"

namespace towr {

/**
 * @brief A spline built from node values and polynomial durations.
 *
 * This class is responsible for combining the optimized node values with
 * the optimized phase durations to construct a sequence of
 * CubicHermitePolynomial. For this it observers whether one of the quantities
 * changed and then updates all the polynomials accordingly.
 */
class Spline : public NodesObserver, public PhaseDurationsObserver {
public:
  using Ptr      = std::shared_ptr<Spline>;
  using VectorXd = Eigen::VectorXd;
  using VecTimes = std::vector<double>;
  using VecPoly  = std::vector<CubicHermitePolynomial>;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  /**
   * @brief Constructs a spline with constant durations.
   * @param nodes_variables The optimized node variables (pos, vel).
   * @param phase_durations The fixed duration of each phase.
   */
  Spline(NodesObserver::SubjectPtr const node_variables,
         const VecTimes& phase_durations);

  /**
   * @brief Constructs a spline with varying/optimized phase durations.
   * @param node_variables The optimized node variables (pos, vel).
   * @param phase_durations Pointer to the changing phase duration variables.
   */
  Spline(NodesObserver::SubjectPtr const node_variables,
         PhaseDurations* const phase_durations);

  virtual ~Spline () = default;

  /**
   * @returns The state of the spline at time t.
   * @param t  The time at which the state of the spline is desired.
   */
  const State GetPoint(double t) const;

  /**
   * @brief Updates the polynomials with the current phase durations.
   *
   * Triggered when phase duration variables (subject) changes (observer pattern).
   */
  void UpdatePhaseDurations();

  /**
   * @brief Updates the polynomials with the current nodes values and durations.
   *
   * Triggered when node values (subject) changes (observer pattern).
   */
  virtual void UpdatePolynomials() override ;



  /**
   * @brief How the spline changes when the node values change.
   * @param t  The time along the spline at which the sensitivity is required.
   * @param dxdt  Whether the derivative of the pos, vel or acc is desired.
   * @return the pxn Jacobian, where:
   *             p: Number of dimensions of the spline
   *             n: Number of optimized node variables.
   */
  Jacobian GetJacobianWrtNodes(double t, Dx dxdt) const;

  /**
   * @brief How the spline position changes when the polynomial durations change.
   * @param t  The time along the spline at which the sensitivity is required.
   * @return the pxn Jacobian, where:
   *             p: Number of dimensions of the spline
   *             n: Number of optimized durations.
   */
  Jacobian GetJacobianOfPosWrtDurations(double t) const;

  /**
   * @returns true if the polynomial at time t is non-changing.
   */
  bool IsConstantPhase(double t) const;

  /**
   * @returns The number of node variables being optimized over.
   */
  int GetNodeVariablesCount() const;

private:
  VecPoly cubic_polys_; ///< the polynomials constructed from the variables.
  VecTimes poly_durations_; ///< the duration of each polynomial


  int GetSegmentID(double t_global, const VecTimes& durations) const;
  std::pair<int,double> GetLocalTime(double t_global, const VecTimes& durations) const;


  Eigen::VectorXd GetDerivativeOfPosWrtPhaseDuration (double t_global) const;





  void Init(const VecTimes& durations);


  // the structure of the nonzero elements of the jacobian with respect to
  // the node values
  mutable Jacobian jac_wrt_nodes_structure_; // all zeros


  // fill_with_zeros is to get sparsity
  void FillJacobian (int poly_id, double t_local, Dx dxdt,
                     Jacobian& jac, bool fill_with_zeros) const;


  void SetPolyFromPhaseDurations(const VecTimes& phase_durations);


};

} /* namespace towr */

#endif /* TOWR_VARIABLES_SPLINE_H_ */
