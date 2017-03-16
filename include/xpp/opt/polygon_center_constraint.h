/**
 @file    polygon_center_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_

#include <xpp/a_constraint.h>
#include <xpp/opt/endeffector_load.h>

namespace xpp {
namespace opt {

/** Ensures that vector represented by lambdas lies in center
  *
  * g(lambda) = (lambda_1-1/m)^2 + ... + (lambda_m-1/m)^2 = 0
  *          =>  lambda_1^2 - 2/m*lambda_1 + ... + lambda_m^2 - 2/m*lambda_m  = -1/m
  *
  * where m = number of contacts at each discrete node
  */
class PolygonCenterConstraint : public AConstraint {
public:

  PolygonCenterConstraint ();
  virtual ~PolygonCenterConstraint ();

  // zmp_ dt at this point must be same discretization as lambda variables
  void Init(const EndeffectorLoad&);

  void UpdateVariables (const OptimizationVariables*) override;
  VectorXd EvaluateConstraint () const override;
  VecBound GetBounds () const override;

  Jacobian GetJacobianWithRespectTo (std::string var_set) const override;

private:
  EndeffectorLoad ee_load_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_POLYGON_CENTER_CONSTRAINT_H_ */
