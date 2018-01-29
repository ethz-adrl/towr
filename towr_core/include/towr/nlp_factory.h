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

#ifndef TOWR_NLP_FACTORY_H_
#define TOWR_NLP_FACTORY_H_

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <towr/variables/spline_holder.h>

#include "robot_model.h"
#include "height_map.h"
#include "parameters.h"

namespace towr {

/** Builds variables, cost and constraints for the legged locomotion problem.
 *
 * Abstracts the entire problem of Trajectory Optimization for walking
 * robots into the formulation of variables, constraints and cost, solvable
 * by any NLP solver.
 */
class NlpFactory {
public:
  using VariablePtrVec   = std::vector<ifopt::VariableSet::Ptr>;
  using ContraintPtrVec  = std::vector<ifopt::ConstraintSet::Ptr>;
  using CostPtrVec       = std::vector<ifopt::CostTerm::Ptr>;
  using EEPos            = std::vector<Eigen::Vector3d>;
  using Vector3d         = Eigen::Vector3d;

  NlpFactory () = default;
  virtual ~NlpFactory () = default;

  /**
   * @returns The ifopt variable sets that will be optimized over.
   */
  VariablePtrVec GetVariableSets();

  /**
   * @returns The ifopt constraints that enforce feasible motions.
   */
  ContraintPtrVec GetConstraints() const;

  /**
   * @returns The ifopt costs to tune the motion.
   */
  ContraintPtrVec GetCosts() const;


  BaseState initial_base_;
  BaseState final_base_;
  EEPos  initial_ee_W_;
  RobotModel model_;
  HeightMap::Ptr terrain_;
  Parameters params_;

  SplineHolder spline_holder_;

private:
  // variables
  std::vector<Nodes::Ptr> MakeBaseVariables() const;
  std::vector<PhaseNodes::Ptr> MakeEndeffectorVariables() const;
  std::vector<PhaseNodes::Ptr> MakeForceVariables() const;
  std::vector<PhaseDurations::Ptr> MakeContactScheduleVariables() const;

  // constraints
  ContraintPtrVec GetConstraint(ConstraintName name) const;
  ContraintPtrVec MakeDynamicConstraint() const;
  ContraintPtrVec MakeRangeOfMotionBoxConstraint() const;
  ContraintPtrVec MakeTotalTimeConstraint() const;
  ContraintPtrVec MakeTerrainConstraint() const;
  ContraintPtrVec MakeForceConstraint() const;
  ContraintPtrVec MakeSwingConstraint() const;
  ContraintPtrVec MakeBaseRangeOfMotionConstraint() const;

  // costs
  CostPtrVec GetCost(const CostName& id, double weight) const;
  CostPtrVec MakeForcesCost(double weight) const;
};

} /* namespace towr */

#endif /* TOWR_NLP_FACTORY_H_ */
