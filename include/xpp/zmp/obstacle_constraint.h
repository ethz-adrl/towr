/**
 @file    obstacle_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace zmp {

class ObstacleConstraint : public AConstraint {
public:
  typedef xpp::utils::StdVecEigen2d FootholdsXY;

  ObstacleConstraint ();
  virtual ~ObstacleConstraint ();

  void UpdateVariables(const ConstraintContainer*) override;

  /** @brief Returns a vector of constraint violations for current variables */
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

private:
  FootholdsXY footholds_;
  const double gap_center_x_ = -0.05; //m
  const double gap_width_x_ = 0.2; //m
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_ */
