/**
 @file    dynamic_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#ifndef TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_

#include <string>
#include <vector>

//#include <ifopt/composite.h>

//#include <xpp_states/cartesian_declarations.h>

#include <towr/models/dynamic_model.h>
#include <towr/variables/spline.h>
#include <towr/variables/angular_state_converter.h>
#include <towr/variables/spline_holder.h>

#include "time_discretization_constraint.h"

namespace towr {

class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using VecTimes = std::vector<double>;
  using Vector3d = Eigen::Vector3d;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  DynamicConstraint (const DynamicModel::Ptr& m,
                     const VecTimes& evaluation_times,
                     const SplineHolder& spline_holder);
  virtual ~DynamicConstraint () = default;

private:

  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;
  std::vector<Spline::Ptr> ee_forces_;
  std::vector<Spline::Ptr> ee_motion_;

  int n_ee_;

  mutable DynamicModel::Ptr model_;
  double gravity_;
  AngularStateConverter converter_;

  int GetRow(int node, Coords6D dimension) const;

  virtual void UpdateConstraintAtInstance(double t, int k, VectorXd& g) const override;
  virtual void UpdateBoundsAtInstance(double t, int k, VecBound& bounds) const override;
  virtual void UpdateJacobianAtInstance(double t, int k, Jacobian&, std::string) const override;

  void UpdateModel(double t) const;
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_DYNAMIC_CONSTRAINT_H_ */
