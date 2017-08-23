/**
 @file    time_discretization_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 30, 2017
 @brief   Declares the TimeDisretizationConstraint class
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_TIME_DISCRETIZATION_CONSTRAINT_H_
#define XPP_XPP_OPT_INCLUDE_XPP_TIME_DISCRETIZATION_CONSTRAINT_H_

#include <string>
#include <vector>

#include <xpp/bound.h>
#include <xpp/composite.h>


namespace xpp {
namespace opt {

/** @brief Constraints evaluated at discretized times along a trajectory.
  */
class TimeDiscretizationConstraint : public Primitive {
public:
  TimeDiscretizationConstraint (double T, double dt, const OptVarsPtr& opt_vars);
  virtual ~TimeDiscretizationConstraint ();

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianWithRespectTo (std::string var_set, Jacobian&) const override;

protected:
  int GetNumberOfNodes() const;

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
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_TIME_DISCRETIZATION_CONSTRAINT_H_ */
