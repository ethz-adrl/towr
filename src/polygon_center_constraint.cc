/**
 @file    polygon_center_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/polygon_center_constraint.h>

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/endeffectors.h>

#include <xpp/bound.h>
#include <xpp/opt/variables/contact_schedule.h>
#include <xpp/opt/variables/endeffector_load.h>

namespace xpp {
namespace opt {

PolygonCenterConstraint::PolygonCenterConstraint (const OptVarsPtr& opt_vars)
{
  name_ = "PolygonCenterConstraint";
  contact_schedule_ = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetSet("contact_schedule"));
  ee_load_          = std::dynamic_pointer_cast<EndeffectorLoad>(opt_vars->GetSet("endeffector_load"));

  int num_constraints = ee_load_->GetNumberOfSegments();
  SetDimensions(opt_vars, num_constraints);
}

PolygonCenterConstraint::~PolygonCenterConstraint ()
{
}

PolygonCenterConstraint::VectorXd
PolygonCenterConstraint::GetValues () const
{
  int num_constraints = GetRows();
  VectorXd g(num_constraints);

  for (int k=0; k<num_constraints; ++k) {
    double g_node = 0.0;
    double t = ee_load_->GetTimeCenterSegment(k);
    int m = contact_schedule_->GetContactCount(t);

    for (auto lambda : ee_load_->GetLoadValuesIdx(k).ToImpl())
      g_node += std::pow(lambda,2) - 2./m * lambda;

    g(k) = g_node;
  }

  return g;
}

VecBound
PolygonCenterConstraint::GetBounds () const
{
  VecBound bounds(GetRows());

  for (int k=0; k<GetRows(); ++k) {
    double t = ee_load_->GetTimeCenterSegment(k);
    int m = contact_schedule_->GetContactCount(t);
    bounds.at(k) = Bound(-1./m, -1./m); // should lie in center of polygon
  }

  return bounds;
}

void
PolygonCenterConstraint::FillJacobianWithRespectTo (std::string var_set,
                                                   Jacobian& jac) const
{
  if (var_set == ee_load_->GetId()) {

    for (int k=0; k<GetRows(); ++k) {
      double t = ee_load_->GetTimeCenterSegment(k);
      int m = contact_schedule_->GetContactCount(t);

      auto lambda_k = ee_load_->GetLoadValuesIdx(k);

      for (auto ee : lambda_k.GetEEsOrdered()) {
        int idx = ee_load_->IndexDiscrete(k,ee);
        jac.coeffRef(k,idx) = 2*(lambda_k.At(ee) - 1./m);
      }
    }
  }
}

} /* namespace opt */
} /* namespace xpp */


