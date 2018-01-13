/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the RangeOfMotionBox class.
 */

#include <towr/constraints/range_of_motion_constraint.h>

#include <memory>
#include <Eigen/Eigen>

#include <towr/variables/variable_names.h>


namespace towr {


RangeOfMotionBox::RangeOfMotionBox (const KinematicModel::Ptr& model,
                                    const OptimizationParameters& params,
                                    const EndeffectorID& ee,
                                    const SplineHolder& spline_holder)
    :TimeDiscretizationConstraint(params.GetTotalTime(),
                                  params.dt_range_of_motion_,
                                  "RangeOfMotionBox-" + std::to_string(ee))
{
  ee_ = ee;

  base_linear_  = spline_holder.GetBaseLinear();
  base_angular_ = spline_holder.GetBaseAngular();
  ee_motion_    = spline_holder.GetEEMotion(ee);
  converter_    = AngularStateConverter(base_angular_);

  max_deviation_from_nominal_ = model->GetMaximumDeviationFromNominal();
  nominal_ee_pos_B_           = model->GetNominalStanceInBase().at(ee);
  SetRows(GetNumberOfNodes()*kDim3d);
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
  AngularStateConverter::MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();
  Vector3d pos_ee_B = b_R_w*(ee_motion_->GetPoint(t).p_ - base_W);

  g.middleRows(GetRow(k, X), kDim3d) = pos_ee_B;
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  using namespace ifopt;

  for (int dim=0; dim<kDim3d; ++dim) {
    Bounds b;
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
  AngularStateConverter::MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();
  int row_start = GetRow(k,X);

  if (var_set == id::base_lin_nodes) {
    jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobianWrtNodes(t, kPos);
  }

  if (var_set == id::base_ang_nodes) {
    Vector3d base_W   = base_linear_->GetPoint(t).p_;
    Vector3d ee_pos_W = ee_motion_->GetPoint(t).p_;
    Vector3d r_W = ee_pos_W - base_W;
    jac.middleRows(row_start, kDim3d) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
  }

  if (var_set == id::EEMotionNodes(ee_)) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_motion_->GetJacobianWrtNodes(t,kPos);
  }

  if (var_set == id::EESchedule(ee_)) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_motion_->GetJacobianOfPosWrtDurations(t);
  }

}

} /* namespace xpp */

