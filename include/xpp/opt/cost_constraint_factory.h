/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include "endeffectors_motion.h"
#include "endeffector_load.h"
#include "center_of_pressure.h"
#include "contact_schedule.h"

#include "motion_parameters.h"

#include <xpp/robot_state_cartesian.h>
#include <xpp/optimization_variables_container.h>

#include <memory>
#include "base_motion.h"

namespace xpp {
namespace opt {

class Constraint;
class Cost;

/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:
  // zmp_ ! possibly make unique_ptr to emphasize that the client now has the
  // Responsibility to delete the memory.
  using ConstraintPtr    = std::shared_ptr<Constraint>;
  using ConstraintPtrVec = std::vector<ConstraintPtr>;
  using CostPtr       = std::shared_ptr<Cost>;
  using Vector2d      = Eigen::Vector2d;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using ComMotionPtr  = std::shared_ptr<BaseMotion>;
  using EEMotionPtr   = std::shared_ptr<EndeffectorsMotion>;
  using ContactSchedulePtr   = std::shared_ptr<ContactSchedule>;
  using EELoadPtr     = std::shared_ptr<EndeffectorLoad>;
  using CopPtr        = std::shared_ptr<CenterOfPressure>;

  using OptVarsContainer = std::shared_ptr<OptimizationVariablesContainer>;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  // zmp_ consider wrapping all Optimization Variables into once class
  void Init(const OptVarsContainer&,
            const ComMotionPtr&, const EEMotionPtr&, const ContactSchedulePtr&,
            const EELoadPtr&, const CopPtr&,
            const MotionTypePtr& params, const RobotStateCartesian& initial_state,
            const StateLin2d& final_state);

  CostPtr GetCost(CostName name) const;
  ConstraintPtrVec GetConstraint(ConstraintName name) const;

private:
  MotionTypePtr params;

  OptVarsContainer opt_vars_;
  // zmp_ remove these!
  ComMotionPtr com_motion;
  EEMotionPtr ee_motion;
  ContactSchedulePtr contact_schedule_;
  EELoadPtr ee_load;
  CopPtr cop;

  RobotStateCartesian initial_geom_state_;
  StateLin2d final_geom_state_;

  // constraints
  ConstraintPtrVec MakeInitialConstraint() const;
  ConstraintPtrVec MakeFinalConstraint() const;
  ConstraintPtrVec MakeJunctionConstraint() const;
  ConstraintPtrVec MakeConvexityConstraint() const;
  ConstraintPtrVec MakeDynamicConstraint() const;
  ConstraintPtrVec MakeRangeOfMotionBoxConstraint() const;
  ConstraintPtrVec MakeStancesConstraints() const;
  ConstraintPtrVec MakeObstacleConstraint() const;
  ConstraintPtrVec MakePolygonCenterConstraint() const;

  // costs
  CostPtr MakeMotionCost() const;

  CostPtr ToCost(const ConstraintPtr& constraint) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */
