/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include "motion_structure.h"
#include "com_motion.h"
#include "endeffectors_motion.h"
#include "endeffector_load.h"

#include "motion_parameters.h"

#include <xpp/state.h>
#include <xpp/variable_set.h>
#include <memory>

namespace xpp {
namespace opt {

class AConstraint;
class ACost;

/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:
  using ConstraintPtr = std::shared_ptr<AConstraint>;
  using CostPtr       = std::shared_ptr<ACost>;
  using Vector2d      = Eigen::Vector2d;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using ComMotionPtr  = std::shared_ptr<ComMotion>;
  using EEMotionPtr   = std::shared_ptr<EndeffectorsMotion>;
  using EELoadPtr     = std::shared_ptr<EndeffectorLoad>;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  // zmp_ consider wrapping all Optimization Variables into once class
  void Init(const ComMotionPtr&, const EEMotionPtr&,
            const EELoadPtr&,
            const MotionStructure&,
            const MotionTypePtr& params, const StateLin2d& initial_state,
            const StateLin2d& final_state);

  // optimization variables with initial values
  VariableSet SplineCoeffVariables() const;
  VariableSet ContactVariables(const Vector2d initial_pos) const;
  VariableSet ConvexityVariables() const;
  VariableSet CopVariables() const;

  CostPtr GetCost(CostName name) const;
  ConstraintPtr GetConstraint(ConstraintName name) const;

private:
  MotionStructure motion_structure;
  MotionTypePtr params;

  ComMotionPtr com_motion;
  EEMotionPtr ee_motion;
  EELoadPtr ee_load;

  StateLin2d initial_geom_state_;
  StateLin2d final_geom_state_;

  // constraints
  ConstraintPtr MakeInitialConstraint() const;
  ConstraintPtr MakeFinalConstraint() const;
  ConstraintPtr MakeJunctionConstraint() const;
  ConstraintPtr MakeConvexityConstraint() const;
  ConstraintPtr MakeSupportAreaConstraint() const;
  ConstraintPtr MakeDynamicConstraint() const;
  ConstraintPtr MakeRangeOfMotionBoxConstraint() const;
  ConstraintPtr MakeFinalStanceConstraint() const;
  ConstraintPtr MakeObstacleConstraint() const;
  ConstraintPtr MakePolygonCenterConstraint() const;

  // costs
  CostPtr MakeMotionCost() const;

  CostPtr ToCost(const ConstraintPtr& constraint) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
