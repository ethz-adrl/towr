/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the RangeOfMotionBox class.
 */

#include <towr/constraints/range_of_motion_constraint.h>

#include <memory>
#include <Eigen/Eigen>

#include <xpp_states/cartesian_declarations.h>

#include <towr/variables/variable_names.h>

namespace xpp {

RangeOfMotionBox::RangeOfMotionBox (const OptimizationParameters& params,
                                    const KinematicModel::Ptr& kinematic_model,
                                    const EndeffectorID& ee,
                                    bool optimize_timings)
    :TimeDiscretizationConstraint(params.GetTotalTime(),
                                  params.dt_range_of_motion_,
                                  "RangeOfMotionBox-" + std::to_string(ee))
{
  ee_ = ee;
  optimize_timings_ = optimize_timings;
  max_deviation_from_nominal_ = kinematic_model->GetMaximumDeviationFromNominal();
  nominal_ee_pos_B_           = kinematic_model->GetNominalStanceInBase().at(ee);
  SetRows(GetNumberOfNodes()*kDim3d);
}

void
RangeOfMotionBox::InitVariableDependedQuantities (const VariablesPtr& x)
{
  base_linear_  = x->GetComponent<Spline>(id::base_linear);
  base_angular_ = x->GetComponent<Spline>(id::base_angular);
  ee_motion_    = x->GetComponent<NodeValues>(id::GetEEMotionId(ee_));

  if (optimize_timings_)
    ee_timings_   = x->GetComponent<ContactSchedule>(id::GetEEScheduleId(ee_));

  converter_ = AngularStateConverter(base_angular_);
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
  using namespace opt;

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

  if (var_set == id::GetEEScheduleId(ee_)) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_timings_->GetJacobianOfPos(t, id::GetEEMotionId(ee_));
  }

  if (var_set == ee_motion_->GetName()) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_motion_->GetJacobian(t,kPos);
  }

  if (var_set == id::base_linear) {
    jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobian(t, kPos);
  }

  if (var_set == id::base_angular) {
    Vector3d base_W   = base_linear_->GetPoint(t).p_;
    Vector3d ee_pos_W = ee_motion_->GetPoint(t).p_;
    Vector3d r_W = ee_pos_W - base_W;
    jac.middleRows(row_start, kDim3d) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
  }
}

} /* namespace xpp */

