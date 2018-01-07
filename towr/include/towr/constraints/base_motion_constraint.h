/**
 @file    base_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Oct 11, 2017
 @brief   Brief description
 */

#ifndef TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_

#include <towr/optimization_parameters.h>
#include <towr/variables/spline.h>

#include "time_discretization_constraint.h"

namespace towr {

/**
 * Keeps the 6D base motion in a specified range.
 *
 */
class BaseMotionConstraint : public TimeDiscretizationConstraint {
public:
  BaseMotionConstraint (const OptimizationParameters& params);
  virtual ~BaseMotionConstraint ();

  virtual void InitVariableDependedQuantities(const VariablesPtr& x) override;

  void UpdateConstraintAtInstance (double t, int k, VectorXd& g) const override;
  void UpdateBoundsAtInstance (double t, int k, VecBound&) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

private:
  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;

  VecBound node_bounds_; ///< same bounds for each discretized node
  int GetRow (int node, int dim) const;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_BASE_MOTION_CONSTRAINT_H_ */
