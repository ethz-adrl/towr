/**
 @file    base_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 11, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_
#define XPP_OPT_INCLUDE_XPP_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_

#include <xpp_opt/constraints/time_discretization_constraint.h>
#include <xpp_opt/optimization_parameters.h>
#include <xpp_opt/variables/spline.h>

namespace xpp {

/**
 * Keeps the 6D base motion in a specified range.
 *
 */
class BaseMotionConstraint : public TimeDiscretizationConstraint {
public:
  BaseMotionConstraint (const Variables& opt_vars,
                        const OptimizationParameters& params);
  virtual ~BaseMotionConstraint ();


  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

private:
  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;

  VecBound node_bounds_; ///< same bounds for each discretized node
  int GetRow (int node, int dim) const;
};

} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_ */
