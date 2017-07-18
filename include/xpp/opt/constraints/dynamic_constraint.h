/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_
#define XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_

#include <memory>
#include <string>

#include <xpp/cartesian_declarations.h>

#include <xpp/opt/variables/polynomial_spline.h>
#include <xpp/opt/angular_state_converter.h>
#include <xpp/opt/dynamic_model.h>

#include "composite.h"
#include "time_discretization_constraint.h"

namespace xpp {
namespace opt {

class BaseMotion;
class EndeffectorsForce;

class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using PolySplinePtr   = std::shared_ptr<PolynomialSpline>;
  using DynamicModelPtr = std::shared_ptr<DynamicModel>;
  using VecTimes        = std::vector<double>;

  DynamicConstraint (const OptVarsPtr& opt_vars,
                     const DynamicModelPtr& m,
                     const VecTimes& base_poly_durations,
                     double T, double dt);
  virtual ~DynamicConstraint ();

private:
  PolySplinePtr base_linear_;
  PolySplinePtr base_angular_;
  std::vector<PolySplinePtr> ee_forces_;
  std::vector<PolySplinePtr> ee_splines_;

  mutable DynamicModelPtr model_;
  AngularStateConverter converter_;

  int GetRow(int node, Coords6D dimension) const;

  virtual void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const override;
  virtual void UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

  void UpdateModel(double t) const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_SRC_DYNAMIC_CONSTRAINT_H_ */
