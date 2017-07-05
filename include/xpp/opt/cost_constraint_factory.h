/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include <memory>

#include <xpp/robot_state_cartesian.h>
#include <xpp/state.h>

#include "linear_spline_equations.h"
#include "motion_parameters.h"
#include <xpp/opt/constraints/composite.h>

namespace xpp {
namespace opt {

class Component;

/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:
  using ConstraintPtr    = std::shared_ptr<Component>;
  using OptVarsContainer = std::shared_ptr<Composite>;
  using MotionParamsPtr  = std::shared_ptr<MotionParameters>;
  using Derivatives      = LinearSplineEquations::MotionDerivatives;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  void Init(const OptVarsContainer&,
            const MotionParamsPtr&,
            const EndeffectorsPos& ee_pos,
            const State3dEuler& initial_base,
            const State3dEuler& final_base);

  ConstraintPtr GetCost(CostName name) const;
  ConstraintPtr GetConstraint(ConstraintName name) const;

private:
  MotionParamsPtr params;

  OptVarsContainer opt_vars_;

  EndeffectorsPos initial_ee_W_;
  State3dEuler initial_base_;
  State3dEuler final_base_;


  // constraints
  ConstraintPtr MakeInitialConstraint() const;
  ConstraintPtr MakeFinalConstraint() const;
  ConstraintPtr MakeJunctionConstraint() const;
  ConstraintPtr MakeDynamicConstraint() const;
  ConstraintPtr MakeRangeOfMotionBoxConstraint() const;
  ConstraintPtr MakeStancesConstraints() const;
//  ConstraintPtr MakePolygonCenterConstraint() const;

  ConstraintPtr MakePolynomialSplineConstraint(const std::string& poly_id,
                                               const StateLin3d state,
                                               double t) const;

  ConstraintPtr MakePolynomialJunctionConstraint(const std::string& poly_id,
                                                 const Derivatives&,
                                                 int skip_every = 1000) const;



  // costs
  ConstraintPtr MakeMotionCost(double weight) const;
  ConstraintPtr MakePolynomialCost(const std::string& poly_id,
                                   const Vector3d& weight_dimensions,
                                   double weight) const;

  ConstraintPtr ToCost(const ConstraintPtr& constraint, double weight) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
