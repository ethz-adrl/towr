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

#ifndef TOWR_NLP_FACTORY_H_
#define TOWR_NLP_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <towr/models/robot_model.h>
#include <towr/optimization_parameters.h>

#include <towr/variables/spline_holder.h>
#include "height_map.h"


namespace towr {


/** Builds all types of constraints/costs for the user.
 *
 * Abstracts the entire problem of Trajectory Optimization for walking
 * robots into the formulation of variables, constraints and cost, solvable
 * by any NLP solver.
 *
 * Implements the factory method, hiding object creation from the client.
 * The client specifies which object it wants, and this class is responsible
 * for the object creation. Factory method is like template method pattern
 * for object creation.
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

  void Init(const OptimizationParameters&,
            const HeightMap::Ptr& terrain,
            const RobotModel& model,
            const EEPos& ee_pos,
            const BaseState& initial_base,
            const BaseState& final_base);

  VariablePtrVec GetVariableSets(SplineHolder&) const;
  ContraintPtrVec GetConstraint(ConstraintName name) const;
  CostPtrVec GetCost(const CostName& id, double weight) const;


private:
  OptimizationParameters params_;
  HeightMap::Ptr terrain_;
  RobotModel model_;

  mutable SplineHolder spline_holder_;


  BaseState initial_base_;
  BaseState final_base_;
  EEPos  initial_ee_W_;




  // variables
  std::vector<NodeVariables::Ptr> MakeBaseVariablesHermite() const;
  std::vector<NodeVariables::Ptr> MakeEndeffectorVariables() const;
  std::vector<NodeVariables::Ptr> MakeForceVariables() const;
  std::vector<ContactSchedule::Ptr> MakeContactScheduleVariables() const;

  // constraints
  ContraintPtrVec MakeDynamicConstraint() const;
  ContraintPtrVec MakeRangeOfMotionBoxConstraint() const;
  ContraintPtrVec MakeTotalTimeConstraint() const;
  ContraintPtrVec MakeTerrainConstraint() const;
  ContraintPtrVec MakeForceConstraint() const;
  ContraintPtrVec MakeSwingConstraint() const;
  ContraintPtrVec MakeBaseRangeOfMotionConstraint() const;

  // costs
  CostPtrVec MakeForcesCost(double weight) const;
//  CostPtrVec MakeMotionCost(double weight) const;
//  CostPtrVec MakePolynomialCost(const std::string& poly_id,
//                                   const Vector3d& weight_dimensions,
//                                   double weight) const;

//  CostPtrVec ToCost(const ConstraintPtr& constraint, double weight) const;

//  std::vector<xpp::EndeffectorID> GetEEIDs() const { return initial_ee_W_.GetEEsOrdered(); };
};

} /* namespace towr */

#endif /* TOWR_NLP_FACTORY_H_ */
