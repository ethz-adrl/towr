/**
 @file    contact_constraints.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 7, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_CONTACT_CONSTRAINTS_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_CONTACT_CONSTRAINTS_H_

#include <memory>
#include <string>

#include <xpp/endeffectors.h>
#include <xpp/opt/bound.h>
#include <xpp/opt/variables/contact_timings.h>
#include <xpp/opt/variables/polynomial_spline.h>

#include <../../xpp_opt/composite.h>
#include <../../xpp_opt/constraints/time_discretization_constraint.h>

namespace xpp {
namespace opt {

class ContactConstraints : public TimeDiscretizationConstraint {
public:
  using PolySplinePtr     = std::shared_ptr<PolynomialSpline>;
  using ContactTimingsPtr = std::shared_ptr<ContactTimings>;

  ContactConstraints (const OptVarsPtr& opt_vars, double T, double dt,
                      EndeffectorID);
  virtual ~ContactConstraints ();


  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

private:
  PolySplinePtr ee_motion_;
  ContactTimingsPtr contact_timings_;

  int GetRow (int node, int dim) const;

  double max_vel_ = 10; // [m/s]

  int constraints_per_node = 2*kDim3d;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONSTRAINTS_CONTACT_CONSTRAINTS_H_ */
