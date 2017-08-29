/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the RangeOfMotionBox class.
 */

#include <xpp/constraints/range_of_motion_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/variables/variable_names.h>

namespace xpp {
namespace opt {

RangeOfMotionBox::RangeOfMotionBox (const OptVarsPtr& opt_vars,
                                    const MotionParamsPtr& params,
                                    const Vector3d& nominal_ee_pos_B,
                                    const Vector3d& max_deviation_from_nominal,
                                    const EndeffectorID& ee)
    :TimeDiscretizationConstraint(params->GetTotalTime(),
                                  params->dt_range_of_motion_,
                                  opt_vars)
{
  SetName("RangeOfMotionBox-" + std::to_string(ee));
  ee_ = ee;
  max_deviation_from_nominal_ = max_deviation_from_nominal;
  nominal_ee_pos_B_           = nominal_ee_pos_B;

  base_linear_  = opt_vars->GetComponent<Spline>(id::base_linear);
  base_angular_ = opt_vars->GetComponent<Spline>(id::base_angular);
  ee_motion_    = opt_vars->GetComponent<NodeValues>(id::GetEEMotionId(ee));
  ee_timings_   = opt_vars->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee));

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
  Vector3d pos_ee_B = b_R_w*(ee_motion_->GetPoint(t).p_ - base_W);

  g.middleRows(GetRow(k, X), kDim3d) = pos_ee_B;
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  for (int dim=0; dim<kDim3d; ++dim) {
    Bound b;
    b += nominal_ee_pos_B_(dim);
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

  if (var_set == ee_timings_->GetName()) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_timings_->GetJacobianOfPos(t, id::GetEEMotionId(ee_));
  }

  if (var_set == ee_motion_->GetName()) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_motion_->GetJacobian(t,kPos);
  }

  if (base_linear_->HoldsVarsetThatIsActiveNow(var_set, t)) {
    jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobian(t, kPos);
  }

  if (base_angular_->HoldsVarsetThatIsActiveNow(var_set, t)) {
    Vector3d base_W   = base_linear_->GetPoint(t).p_;
    Vector3d ee_pos_W = ee_motion_->GetPoint(t).p_;
    Vector3d r_W = ee_pos_W - base_W;
    jac.middleRows(row_start, kDim3d) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
  }
}

} /* namespace opt */
} /* namespace xpp */
