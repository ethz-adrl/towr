/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include <memory>
#include <string>
#include <vector>

#include <ifopt/composite.h>
#include <ifopt/leaves.h>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>

#include <xpp_opt/height_map.h>
#include <xpp_opt/models/robot_model.h>
#include <xpp_opt/optimization_parameters.h>


namespace xpp {


/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:

  using ComponentPtr     = std::shared_ptr<opt::Component>;
  using ConstraintPtr    = std::shared_ptr<opt::ConstraintSet>;
  using VariablePtrVec   = std::vector<opt::VariableSet::Ptr>;
  using ContraintPtrVec  = std::vector<ConstraintPtr>;
  using CostPtr          = std::shared_ptr<opt::CostTerm>;
  using CostPtrVec       = std::vector<CostPtr>;
  using OptVarsContainer = std::shared_ptr<opt::Composite>;
  using MotionParamsPtr  = std::shared_ptr<OptimizationParameters>;
  using Derivatives      = std::vector<MotionDerivative>;

  CostConstraintFactory () = default;
  virtual ~CostConstraintFactory () = default;

  void Init(const MotionParamsPtr&,
            const HeightMap::Ptr& terrain,
            const RobotModel& model,
            const EndeffectorsPos& ee_pos,
            const State3dEuler& initial_base,
            const State3dEuler& final_base);

  VariablePtrVec GetVariableSets() const;
  ContraintPtrVec GetConstraint(ConstraintName name) const;
  CostPtrVec GetCost(const CostName& id, double weight) const;

private:
  MotionParamsPtr params_;
  HeightMap::Ptr terrain_;
  RobotModel model_;


  EndeffectorsPos initial_ee_W_;
  State3dEuler initial_base_;
  State3dEuler final_base_;


  // variables
  VariablePtrVec MakeBaseVariablesCoeff() const;
  VariablePtrVec MakeBaseVariablesHermite() const;
  VariablePtrVec MakeEndeffectorVariables() const;
  VariablePtrVec MakeForceVariables() const;
  VariablePtrVec MakeContactScheduleVariables(const VariablePtrVec& ee_motion,
                                              const VariablePtrVec& ee_force) const;

  // constraints
  ContraintPtrVec MakeStateConstraint() const;
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

  std::vector<EndeffectorID> GetEEIDs() const { return initial_ee_W_.GetEEsOrdered(); };
};

} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
