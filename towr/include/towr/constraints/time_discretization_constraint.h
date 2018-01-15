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

#ifndef TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_

#include <string>
#include <vector>

#include <ifopt/constraint_set.h>

namespace towr {

/** @brief Constraints evaluated at discretized times along a trajectory.
  */
class TimeDiscretizationConstraint : public ifopt::ConstraintSet {
public:
  using EvaluationTimes = std::vector<double>;
  using Bounds          = ifopt::Bounds;

  TimeDiscretizationConstraint (double T, double dt,const std::string& name);
  TimeDiscretizationConstraint (const EvaluationTimes&,const std::string& name);
  virtual ~TimeDiscretizationConstraint ();

  Eigen::VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

protected:
  int GetNumberOfNodes() const;
  EvaluationTimes dts_; ///< times at which the constraint is evaluated.

private:
  /** Sets the constraint value a specific time t, corresponding to node k.
   */
  virtual void UpdateConstraintAtInstance(double t, int k, VectorXd&) const = 0;

  /** Sets upper/lower bound a specific time t, corresponding to node k.
   */
  virtual void UpdateBoundsAtInstance(double t, int k, VecBound&) const = 0;

  /** Sets Jacobian rows at a specific time t, corresponding to node k.
   */
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const = 0;

};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_ */
