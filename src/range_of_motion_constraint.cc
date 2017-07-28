/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the RangeOfMotionBox class.
 */

#include <xpp/opt/constraints/range_of_motion_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/variables/node_values.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

RangeOfMotionBox::RangeOfMotionBox (const OptVarsPtr& opt_vars,
                                    const MotionParamsPtr& params,
                                    const EndeffectorID& ee)
    :TimeDiscretizationConstraint(params->GetTotalTime(),
                                  params->dt_range_of_motion_,
                                  opt_vars)
{
  SetName("RangeOfMotionBox-" + std::to_string(ee));
  max_deviation_from_nominal_ = params->GetMaximumDeviationFromNominal();
  nominal_ee_pos_B            = params->GetNominalStanceInBase().At(ee);

  auto base_poly_durations = params->GetBasePolyDurations();

  base_linear_  = Spline::BuildSpline(opt_vars, id::base_linear,  base_poly_durations);
  base_angular_ = Spline::BuildSpline(opt_vars, id::base_angular, base_poly_durations);
  // zmp_ hide node value implementation detail again, remove cast
  ee_spline_    = std::dynamic_pointer_cast<EEMotionType>(Spline::BuildSpline(opt_vars, id::GetEEId(ee), {}));
  ee_timings_   = std::dynamic_pointer_cast<ContactSchedule>(opt_vars->GetComponent(id::GetEEScheduleId(ee)));

  SetRows(GetNumberOfNodes()*kDim3d);
  converter_ = AngularStateConverter(base_angular_);
}

RangeOfMotionBox::~RangeOfMotionBox ()
{
}

int
RangeOfMotionBox::GetRow (int node, int dim) const
{
  return node*kDim3d + dim;
}

void
RangeOfMotionBox::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
  Vector3d base_W = base_linear_->GetPoint(t).p_;
  MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();

  Vector3d pos_ee_B = b_R_w*(ee_spline_->GetPoint(t).p_ - base_W);
  g.middleRows(GetRow(k, X), kDim3d) = pos_ee_B;
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  for (int dim=0; dim<kDim3d; ++dim) {
    Bound b;
    b += nominal_ee_pos_B(dim);
    b.upper_ += max_deviation_from_nominal_(dim);
    b.lower_ -= max_deviation_from_nominal_(dim);
    bounds.at(GetRow(k,dim)) = b;
  }
}

void
RangeOfMotionBox::UpdateJacobianAtInstance (double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();
  int row_start = GetRow(k,X);

  if (ee_spline_->DoVarAffectCurrentState(var_set,t)) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_spline_->GetJacobian(t,kPos);
  }

//   spring_clean_ first use of time values in derivatives, this is the goal!
  if (var_set == ee_timings_->GetName()) {
    VectorXd duration_deriv = ee_spline_->GetDerivativeOfPosWrtDuration(t);
    VectorXd vel = ee_spline_->GetPoint(t).v_;
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_timings_->GetJacobianOfPos(duration_deriv, vel, t);
  }

  if (base_linear_->DoVarAffectCurrentState(var_set,t)) {
    jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobian(t, kPos);
  }

  if (base_angular_->DoVarAffectCurrentState(var_set,t)) {
    Vector3d base_W   = base_linear_->GetPoint(t).p_;
    Vector3d ee_pos_W = ee_spline_->GetPoint(t).p_;
    Vector3d r_W = ee_pos_W - base_W;
    jac.middleRows(row_start, kDim3d) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
  }
}

} /* namespace opt */
} /* namespace xpp */
