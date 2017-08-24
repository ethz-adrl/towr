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
#include <xpp/bound.h>
#include <xpp/cartesian_declarations.h>
#include <xpp/composite.h>
#include <xpp/dynamic_model.h>
#include <xpp/variables/contact_schedule.h>
#include <xpp/variables/spline.h>

#include "time_discretization_constraint.h"


namespace xpp {
namespace opt {

class DynamicConstraint : public TimeDiscretizationConstraint {
public:
  using DynamicModelPtr = std::shared_ptr<DynamicModel>;
  using VecTimes        = std::vector<double>;
  using SplineT         = std::shared_ptr<Spline>;
  using SchedulePtr     = std::shared_ptr<ContactSchedule>;

  DynamicConstraint (const OptVarsPtr& opt_vars,
                     const DynamicModelPtr& m,
                     double T, double dt);
  virtual ~DynamicConstraint ();

private:
  SplineT base_linear_;
  SplineT base_angular_;
  std::vector<SplineT> ee_forces_;
  std::vector<SplineT> ee_motion_;
  std::vector<SchedulePtr> ee_timings_;

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
