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

#ifndef TOWR_VARIABLES_PHASE_NODES_H_
#define TOWR_VARIABLES_PHASE_NODES_H_

#include "nodes.h"

namespace towr {

/**
 * @brief Nodes that are associated to either swing or stance phases.
 *
 * In the node values, not every node is an optimization variable, but two
 * consecutive nodes forming a e.g. stance position spline belong to the
 * same optimization variable. This is because a foot in stance cannot
 * move (or a force in flight must be zero). This makes the number of
 * optimization variables less than the total node values.
 */
class PhaseNodes : public Nodes {
public:
  using Ptr       = std::shared_ptr<PhaseNodes>;
  using OptNodeIs = int;
  using NodeIds   = std::vector<int>;

  enum Type {Force, Motion};

  /**
   * @brief Holds semantic information each polynomial in spline.
   */
  struct PolyInfo {
    int phase_; ///< The phase ID this polynomial represents.
    int poly_in_phase_; ///< is this the 1st, 2nd, ... polynomial or this phase.
    int n_polys_in_phase_; ///< the number of polynomials used for this phase.
    bool is_constant_; ///< Does this polynomial represent a constant phase.
    PolyInfo(int phase, int poly_in_phase, int n_polys_in_phase, bool is_const);
  };

  /**
   * @brief Constructs a variable set of node variables.
   * @param phase_count  The number of phases (swing, stance) to represent.
   * @param in_contact__start  Whether the first node belongs to a stance phase.
   * @param var_name  The name given to this set of optimization variables.
   * @param n_polys_in_changing_phase  How many polynomials should be used to
   *                                   paramerize each non-constant phase.
   * @param type  These nodes represent either force evolution or foot motions.
   */
  PhaseNodes (int phase_count, bool in_contact_start, const std::string& var_name,
              int n_polys_in_changing_phase, Type type);

  virtual ~PhaseNodes() = default;

  virtual std::vector<IndexInfo> GetNodeInfoAtOptIndex(int idx) const override;

  /**
   * @returns the value of the first node of the phase.
   */
  Eigen::Vector3d GetValueAtStartOfPhase(int phase) const;

  /**
   * @returns the ID of the first node in the phase.
   */
  int GetNodeIDAtStartOfPhase(int phase) const;

  /**
   * @returns The phase ID belonging to node with node_id.
   *
   * Only makes sense if left and right polynomial belong to same phase.
   */
  int GetPhase(int node_id) const;

  /**
   * @brief node is constant if either left or right polynomial belongs to a
   * constant phase.
   */
  virtual bool IsConstantNode(int node_id) const;

  /**
   * @brief The indices of those nodes that don't belong to a constant phase.
   *
   * For forces nodes these are the stance phases (can produce force), and for
   * feet these are the swing phases (can move end-effector).
   */
  NodeIds GetIndicesOfNonConstantNodes() const;

  /**
   * @brief Converts durations of swing and stance phases to polynomial durations.
   * @param phase_durations  The durations of alternating swing and stance phases.
   * @return  The durations of each polynomial, where multiple polynomials can
   *          be used to represent one phase.
   */
  virtual VecDurations
  ConvertPhaseToPolyDurations(const VecDurations& phase_durations) const;

  /**
   * @brief How a change in the phase duration affects the polynomial duration.
   * @param polynomial_id  The ID of the polynomial within the spline.
   *
   * If a phase is represented by multiple (3) equally timed polynomials, then
   * T_poly = 1/3 * T_phase.
   * The derivative of T_poly is then 1/3.
   */
  virtual double
  GetDerivativeOfPolyDurationWrtPhaseDuration(int polynomial_id) const;

  /**
   * @brief How many polynomials in the current phase come before.
   * @param polynomial_id  The ID of the polynomial within the spline.
   *
   * If a phase is represented by multiple (3) polynomials, and the current
   * polynomial corresponds to the third one in the phase, then 2 polynomials
   * come before it.
   */
  virtual int
  GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const;

  /**
   * @brief Is the polynomial constant, so not changing the value.
   * @param polynomial_id The ID of the polynomial within the spline.
   */
  virtual bool IsInConstantPhase(int polynomial_id) const;

private:
  std::vector<PolyInfo> polynomial_info_;

  // maps from the nodes that are actually optimized over to all the nodes.
  // Optimized nodes are sometimes used twice in a constant phase.
  // This is where the constant phases are enforced.
  std::map<OptNodeIs, NodeIds > optnode_to_node_;

  /**
   * @returns the ID of the polynomial at the start of phase phase.
   */
  int GetPolyIDAtStartOfPhase(int phase) const;

  static std::map<OptNodeIs, NodeIds>
  GetOptNodeToNodeMappings(const std::vector<PolyInfo>&);

  /**
   * @brief Sets the bounds on the node variables to model foot motions.
   *
   * For this the velocity of the stance nodes is bounds to zero.
   */
  void SetBoundsEEMotion();

  /**
   * @brief Sets the bounds on the node variables to model foot forces.
   *
   * For this the force for nodes representing swing-phases is set to zero.
   */
  void SetBoundsEEForce();

  std::vector<int> GetAdjacentPolyIds(int node_id) const;

};
} /* namespace towr */

#endif /* TOWR_VARIABLES_PHASE_NODES_H_ */
