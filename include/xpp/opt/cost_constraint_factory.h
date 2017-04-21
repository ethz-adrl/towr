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

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  void Init(const OptVarsContainer&,
            const MotionParamsPtr& params,
            const RobotStateCartesian& initial_state,
            const StateLin2d& final_state);

  ConstraintPtr GetCost(CostName name) const;
  ConstraintPtr GetConstraint(ConstraintName name) const;

private:
  MotionParamsPtr params;

  OptVarsContainer opt_vars_;
  RobotStateCartesian initial_geom_state_;
  StateLin2d final_geom_state_;

  LinearSplineEquations spline_eq_;

  // constraints
  ConstraintPtr MakeInitialConstraint() const;
  ConstraintPtr MakeFinalConstraint() const;
  ConstraintPtr MakeJunctionConstraint() const;
  ConstraintPtr MakeDynamicConstraint() const;
  ConstraintPtr MakeRangeOfMotionBoxConstraint() const;
  ConstraintPtr MakeStancesConstraints() const;
  ConstraintPtr MakePolygonCenterConstraint() const;

  // costs
  ConstraintPtr MakeMotionCost(double weight) const;
  ConstraintPtr ToCost(const ConstraintPtr& constraint, double weight) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
