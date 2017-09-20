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
#include <vector>

#include <xpp/angular_state_converter.h>
#include <xpp/cartesian_declarations.h>
#include <xpp/composite.h>
#include <xpp/models/dynamic_model.h>
#include <xpp/nlp_bound.h>
#include <xpp/variables/contact_schedule.h>
#include <xpp/variables/spline.h>

#include "time_discretization_constraint.h"


namespace xpp {
namespace opt {

class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using VecTimes        = std::vector<double>;

  DynamicConstraint (const OptVarsPtr& opt_vars,
                     const DynamicModel::Ptr& m,
                     const std::vector<double>& evaluation_times);
  virtual ~DynamicConstraint ();

private:
  Spline::Ptr base_linear_;
  Spline::Ptr base_angular_;
  std::vector<NodeValues::Ptr> ee_forces_;
  std::vector<NodeValues::Ptr> ee_motion_;
  std::vector<ContactSchedule::Ptr> ee_timings_;

  mutable DynamicModel::Ptr model_;
  double gravity_;
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
