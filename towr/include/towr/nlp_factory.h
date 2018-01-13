/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef TOWR_NLP_FACTORY_H_
#define TOWR_NLP_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <towr/variables/base_state.h>

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

  NlpFactory () = default;
  virtual ~NlpFactory () = default;

  void Init(const OptimizationParameters&,
            const HeightMap::Ptr& terrain,
            const RobotModel& model,
            const NewEEPos& ee_pos,
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


  BaseState new_initial_base_;
  BaseState new_final_base_;
  NewEEPos  new_initial_ee_W_;




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
