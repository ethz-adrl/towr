/**
 @file    time_discretization_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 30, 2017
 @brief   Declares the TimeDisretizationConstraint class
 */

#ifndef TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_

#include <string>
#include <vector>

#include <ifopt/constraint_set.h>

namespace towr {

/** @brief Constraints evaluated at discretized times along a trajectory.
  */
class TimeDiscretizationConstraint : public ifopt::ConstraintSet {
public:
  using EvaluationTimes = std::vector<double>;
  using Bounds          = ifopt::Bounds;

  TimeDiscretizationConstraint (double T, double dt,const std::string& name);
  TimeDiscretizationConstraint (const EvaluationTimes&,const std::string& name);
  virtual ~TimeDiscretizationConstraint ();

  Eigen::VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

protected:
  int GetNumberOfNodes() const;
  EvaluationTimes dts_; ///< times at which the constraint is evaluated.

private:
  /** Sets the constraint value a specific time t, corresponding to node k.
   */
  virtual void UpdateConstraintAtInstance(double t, int k, VectorXd&) const = 0;

  /** Sets upper/lower bound a specific time t, corresponding to node k.
   */
  virtual void UpdateBoundsAtInstance(double t, int k, VecBound&) const = 0;

  /** Sets Jacobian rows at a specific time t, corresponding to node k.
   */
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const = 0;

};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_TIME_DISCRETIZATION_CONSTRAINT_H_ */
