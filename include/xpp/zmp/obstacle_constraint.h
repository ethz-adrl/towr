/**
 @file    obstacle_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/utils/eigen_std_conversions.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/ellipse.h>

namespace xpp {
namespace opt {

class ObstacleConstraint : public AConstraint {
public:
  using FootholdsXY = xpp::utils::StdVecEigen2d;
  using Contacts = xpp::hyq::SupportPolygonContainer;

  ObstacleConstraint ();
  virtual ~ObstacleConstraint ();

  void Init(const Contacts&);
  virtual void UpdateVariables(const OptimizationVariables*) override final;

protected:
  FootholdsXY footholds_;
  Contacts contacts_;
  const double gap_center_x_ = 0.45; //m
  const double gap_width_x_ = 0.15; //m
};

class ObstacleLineStrip : public ObstacleConstraint {
public:
  ObstacleLineStrip ();
  virtual ~ObstacleLineStrip ();

  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;
  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;
};

class ObstacleEllipse : public ObstacleConstraint {
public:
  ObstacleEllipse ();
  virtual ~ObstacleEllipse ();

  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;
  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  Ellipse ellipse_;
};


} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_ */
