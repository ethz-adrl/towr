/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the RangeOfMotionBox class.
 */

#include <xpp/opt/constraints/range_of_motion_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/state.h>

#include <xpp/opt/variables/polynomial_spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

RangeOfMotionBox::RangeOfMotionBox (const OptVarsPtr& opt_vars,
                                    double dt,
                                    const Vector3d& max_dev_B,
                                    const Vector3d& nominal_ee_B,
                                    const EndeffectorID& ee,
                                    double T)
    :TimeDiscretizationConstraint(T, dt, opt_vars)
{
  SetName("RangeOfMotionBox-" + std::to_string(ee));
  max_deviation_from_nominal_ = max_dev_B;
  nominal_ee_pos_B            = nominal_ee_B;

  std::string id_ee_motion = id::endeffectors_motion+std::to_string(ee);
  base_linear_  = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_linear));
  base_angular_ = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_angular));
  ee_spline_    = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id_ee_motion));

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

  if (var_set == ee_spline_->GetName()) {
    jac.middleRows(row_start, kDim3d) = b_R_w*ee_spline_->GetJacobian(t,kPos);
    // zmp_ add separate jacobian w.r.t. timings of each polynomial.
    // keep top line untouched for backwards compatiblity.
  }

//  if (var_set == contact_timings_->GetName()) {
//    jac.middleRows(row_start, kDim3d) = b_R_w*ee_spline_->GetJacobian(t,kPos);
//    // zmp_ add separate jacobian w.r.t. timings of each polynomial.
//    // keep top line untouched for backwards compatiblity.
//  }



  if (var_set == base_linear_->GetName()) {
    jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobian(t, kPos);
  }

  if (var_set == base_angular_->GetName()) {
    Vector3d base_W   = base_linear_->GetPoint(t).p_;
    Vector3d ee_pos_W = ee_spline_->GetPoint(t).p_;
    Vector3d r_W = ee_pos_W - base_W;
    jac.middleRows(row_start, kDim3d) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
  }
}

} /* namespace opt */
} /* namespace xpp */
