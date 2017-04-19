/**
 @file    time_discretization_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 30, 2017
 @brief   Declares the TimeDisretizationConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_TIME_DISCRETIZATION_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_TIME_DISCRETIZATION_CONSTRAINT_H_

#include <vector>
#include "constraint.h"

namespace xpp {
namespace opt {

/** @brief Constraints evaluated at discretized times along a trajectory.
  */
class TimeDiscretizationConstraint : public ConstraintLeaf {
public:
  TimeDiscretizationConstraint (double T, double dt, int constraints_per_time,
                                const OptVarsPtr& opt_vars);
  virtual ~TimeDiscretizationConstraint ();

  VectorXd GetConstraintValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

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

  std::vector<double> dts_; ///< discretized times
  int GetNumberOfNodes() const;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_TIME_DISCRETIZATION_CONSTRAINT_H_ */
