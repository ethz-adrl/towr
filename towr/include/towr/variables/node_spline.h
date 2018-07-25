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

#ifndef TOWR_TOWR_SRC_NODE_SPLINE_H_
#define TOWR_TOWR_SRC_NODE_SPLINE_H_

#include <memory>
#include <Eigen/Sparse>

#include "spline.h"
#include "nodes_observer.h"

namespace towr {

/**
 * @brief A spline built from node values and fixed polynomial durations.
 *
 * This class is responsible for combining the optimized node values with
 * fixed polynomial durations to construct a sequence of CubicHermitePolynomial.
 * For this it observes whether the node values change and then updates all the
 * polynomials accordingly.
 */
class NodeSpline : public Spline, public NodesObserver {
public:
  using Ptr = std::shared_ptr<NodeSpline>;
  using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  /**
   * @brief Constructs a spline with constant durations.
   * @param nodes_variables The optimized node variables (pos, vel).
   * @param phase_durations The fixed duration of each phase.
   */
  NodeSpline(NodeSubjectPtr const node_variables,
             const VecTimes& polynomial_durations);
  ~NodeSpline() = default;

  /**
   * @brief Called by subject to update the polynomials with new node values.
   */
  void UpdateNodes();

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
   * @brief How the spline changes when the node values change.
   * @param poly_id  Polynomial for which the sensitivity is desired.
   * @param t_local  Local time in that specific polynomial.
   * @param dxdt  Whether the derivative of the pos, vel or acc is desired.
   * @return the pxn Jacobian, where:
   *             p: Number of dimensions of the spline
   *             n: Number of optimized node variables.
   */
  Jacobian GetJacobianWrtNodes(int poly_id, double t_local, Dx dxdt) const;

  /**
   * @returns The number of node variables being optimized over.
   */
  int GetNodeVariablesCount() const;

  /**
   * @brief How the spline position changes when the polynomial durations change.
   * @param t  The time along the spline at which the sensitivity is required.
   * @return the pxn Jacobian, where:
   *             p: Number of dimensions of the spline
   *             n: Number of optimized durations.
   */
  virtual Jacobian
  GetJacobianOfPosWrtDurations(double t) const { assert(false); } // durations are fixed here

protected:
  /**
   * The size and non-zero elements of the Jacobian of the position w.r.t nodes.
   */
  mutable Jacobian jac_wrt_nodes_structure_;

  /**
   * @brief Fills specific elements of the Jacobian with respect to nodes.
   * @param poly_id  The ID of the polynomial for which to get the sensitivity.
   * @param t_local  The time passed in the polynomial with poly_id.
   * @param dxdt     Whether the function is position, velocity or acceleration.
   * @param jac[in/out] The correctly sized Jacobian to fill.
   * @param fill_with_zeros True if only sparsity pattern should be set.
   */
  void FillJacobianWrtNodes (int poly_id, double t_local, Dx dxdt,
                             Jacobian& jac, bool fill_with_zeros) const;
};

} /* namespace towr */

#endif /* TOWR_TOWR_SRC_NODE_SPLINE_H_ */
