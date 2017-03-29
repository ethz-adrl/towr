/**
 @file    obstacle_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_OBSTACLE_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_OBSTACLE_CONSTRAINT_H_

#include "../constraint.h"
#include "ellipse.h"
#include "eigen_std_conversions.h"
#include "endeffectors_motion.h"

namespace xpp {
namespace opt {

class ObstacleConstraint : public Constraint {
public:
  ObstacleConstraint ();
  virtual ~ObstacleConstraint ();

  virtual void UpdateVariables(const OptimizationVariables*) override final;

protected:
  StdVecEigen2d footholds_;
  const double gap_center_x_ = 0.45; //m
  const double gap_width_x_ = 0.15; //m
};

class ObstacleLineStrip : public ObstacleConstraint {
public:
  ObstacleLineStrip ();
  virtual ~ObstacleLineStrip ();

  VectorXd UpdateConstraintValues () const override;
  VecBound UpdateBounds () const override;
  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;
};

class ObstacleEllipse : public ObstacleConstraint {
public:
  ObstacleEllipse ();
  virtual ~ObstacleEllipse ();

  VectorXd UpdateConstraintValues () const override;
  VecBound UpdateBounds () const override;
  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  Ellipse ellipse_;
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_OBSTACLE_CONSTRAINT_H_ */
