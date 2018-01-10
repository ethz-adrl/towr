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

#include <ifopt/leaves.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>

#include <towr/constraints/height_map.h>
#include <towr/models/robot_model.h>
#include <towr/optimization_parameters.h>

#include <towr/variables/spline_holder.h>


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
  using EndeffectorsPos  = xpp::EndeffectorsPos;
  using State3dEuler     = xpp::State3dEuler;


  NlpFactory () = default;
  virtual ~NlpFactory () = default;

  void Init(const OptimizationParameters&,
            const HeightMap::Ptr& terrain,
            const RobotModel& model,
            const xpp::EndeffectorsPos& ee_pos,
            const xpp::State3dEuler& initial_base,
            const xpp::State3dEuler& final_base);

  VariablePtrVec GetVariableSets() const;
  ContraintPtrVec GetConstraint(ConstraintName name) const;
  CostPtrVec GetCost(const CostName& id, double weight) const;

  // smell move to new class at some point
  mutable SplineHolder spline_holder_;

private:
  OptimizationParameters params_;
  HeightMap::Ptr terrain_;
  RobotModel model_;
  EndeffectorsPos initial_ee_W_;
  State3dEuler initial_base_;
  State3dEuler final_base_;



  // variables
  std::vector<NodeValues::Ptr> MakeBaseVariablesHermite() const;
  std::vector<NodeValues::Ptr> MakeEndeffectorVariables() const;
  std::vector<NodeValues::Ptr> MakeForceVariables() const;
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

  std::vector<xpp::EndeffectorID> GetEEIDs() const { return initial_ee_W_.GetEEsOrdered(); };
};

} /* namespace towr */

#endif /* TOWR_NLP_FACTORY_H_ */
