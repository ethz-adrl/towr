/**
 @file    a_foothold_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Declares an abstract FootholdCost class and one concrete derivation.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_COST_H_

#include <xpp/hyq/support_polygon_container.h>
#include "a_constraint.h"
#include <xpp/hyq/foothold.h>
#include <memory>

namespace xpp {
namespace opt {

class ARobotInterface;


/** Base class for Costs associated only with the foothold positions
  *
  * This class is responsible for updating the current position of the footholds
  * in order to then calculate costs based on these (move towards goal, avoid
  * obstacles, ...).
  */
class AFootholdConstraint : public AConstraint {
public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  AFootholdConstraint ();
  virtual ~AFootholdConstraint ();

  void UpdateVariables(const OptimizationVariables*) override;

protected:
  void Init(const SupportPolygonContainer&);
  SupportPolygonContainer supp_polygon_container_;
};


/** This class assigns a cost if the center of the final footholds is far
  * away from the desired goal position.
  */
class FootholdFinalStanceConstraint : public AFootholdConstraint {
public:
  typedef Eigen::Vector2d Vector2d;
  using RobotPtrU = std::unique_ptr<ARobotInterface>;

  FootholdFinalStanceConstraint(const Vector2d& goal_xy, const SupportPolygonContainer&,
                                RobotPtrU);
  virtual ~FootholdFinalStanceConstraint();

  virtual VectorXd EvaluateConstraint () const override;
  virtual Jacobian GetJacobianWithRespectTo(std::string var_set) const override;
  virtual VecBound GetBounds() const override;

private:
  Vector2d GetFootToNominalInWorld(const hyq::Foothold& foot_W) const;

  Vector2d goal_xy_;
  RobotPtrU robot_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_COST_H_ */
