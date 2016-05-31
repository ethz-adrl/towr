/*
 * zmp_constraint.h
 *
 *  Created on: May 25, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZMP_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZMP_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/zmp_constraint_builder.h>

#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

class ZmpConstraint : public IObserver, public AConstraint {
public:
  typedef OptimizationVariables::StdVecEigen2d FootholdsXY;
  typedef xpp::utils::MatVec MatVec;
  typedef xpp::hyq::SupportPolygonContainer SupportPolygonContainer;

  ZmpConstraint (OptimizationVariables& subject);
  virtual ~ZmpConstraint () {};

  void Init(const ContinuousSplineContainer&,
            const SupportPolygonContainer&,
            double walking_height);

  void Update () override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

private:
  VectorXd x_coeff_;
  FootholdsXY footholds_;

  SupportPolygonContainer supp_polygon_container_;

  ZmpConstraintBuilder zmp_constraint_builder_;

  OptimizationVariables* subject_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_ZMP_CONSTRAINT_H_ */
