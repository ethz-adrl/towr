/**
 @file    a_foothold_cost.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 8, 2016
 @brief   Declares an abstract FootholdCost class and one concrete derivation.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_COST_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_COST_H_

#include <xpp/hyq/support_polygon_container.h>
#include "../opt/a_cost.h"

namespace xpp {
namespace opt {

/** Base class for Costs associated only with the foothold positions
  *
  * This class is responsible for updating the current position of the footholds
  * in order to then calculate costs based on these (move towards goal, avoid
  * obstacles, ...).
  */
class AFootholdCost : public ACost {
public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  AFootholdCost ();
  virtual ~AFootholdCost ();

  void UpdateVariables(const OptimizationVariables*) override;

protected:
  void Init(const SupportPolygonContainer&);
  SupportPolygonContainer supp_polygon_container_;
};


/** This class assigns a cost if the center of the final footholds is far
  * away from the desired goal position.
  */
class FootholdGoalCost : public AFootholdCost {
public:
  typedef Eigen::Vector2d Vector2d;

  void Init(const Vector2d& goal_xy, const SupportPolygonContainer&);
  double EvaluateCost () const override;

private:
  Vector2d goal_xy_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_FOOTHOLD_COST_H_ */
