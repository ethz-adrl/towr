/*
 * zmp_constraint.h
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZMP_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZMP_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/motion_structure.h>
#include <xpp/zmp/zmp_constraint_builder.h>
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class OptimizationVariablesInterpreter;

class ZmpConstraint : public AConstraint {
public:
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;
  using Contacts = xpp::hyq::SupportPolygonContainer;
  typedef xpp::utils::StdVecEigen2d FootholdsXY;

  ZmpConstraint ();
  virtual ~ZmpConstraint () {};

  void Init(const MotionStructure&, const ComMotion&, const Contacts&, double walking_height);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  ZmpConstraintBuilder zmp_constraint_builder_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZMP_CONSTRAINT_H_ */
