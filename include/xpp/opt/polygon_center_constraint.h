/**
 @file    polygon_center_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_

#include "a_constraint.h"

namespace xpp {
namespace opt {

class MotionStructure;

class PolygonCenterConstraint : public AConstraint {
public:
  PolygonCenterConstraint ();
  virtual ~PolygonCenterConstraint ();

  void Init(const MotionStructure&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  std::vector<int> n_contacts_per_node_;
  VectorXd lambdas_;
  Jacobian jac_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_ */
