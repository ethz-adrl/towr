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

#ifndef TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_

#include <string>

#include <ifopt/constraint_set.h>

#include <towr/variables/phase_nodes.h>
#include <towr/height_map.h>

namespace towr {

/** Ensures the end-effector force lies inside friction cone.
 *
 * Attention: This is enforced only at the spline nodes.
 */
class ForceConstraint : public ifopt::ConstraintSet {
public:
  using EndeffectorID = uint;
  using Vector3d = Eigen::Vector3d;

  ForceConstraint (const HeightMap::Ptr& terrain,
                   double force_limit_in_normal_direction,
                   EndeffectorID ee);
  virtual ~ForceConstraint () = default;

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  /** @brief Returns a vector of constraint violations for current variables \c x_coeff. */
  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;


private:
  PhaseNodes::Ptr ee_force_;
  PhaseNodes::Ptr ee_motion_;

  HeightMap::Ptr terrain_;
  double force_limit_normal_direction_;
  double mu_; // friction coeff
  int n_constraints_per_node_;

  EndeffectorID ee_;
  std::vector<int> pure_stance_force_node_ids_;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_FORCE_CONSTRAINT_H_ */
