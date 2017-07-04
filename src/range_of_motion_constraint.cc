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
                                    const MaxDevXY& dev,
                                    const NominalStance& nom,
                                    double T)
    :TimeDiscretizationConstraint(T, dt, opt_vars)
{
  SetName("RangeOfMotionBox");
  max_deviation_from_nominal_ = dev;
  nominal_stance_ = nom;

  base_linear_  = std::dynamic_pointer_cast<PolynomialSpline>  (opt_vars->GetComponent(id::base_linear));
  base_angular_ = std::dynamic_pointer_cast<PolynomialSpline>  (opt_vars->GetComponent(id::base_angular));

  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    std::string id = id::endeffectors_motion+std::to_string(ee);
    ee_splines_.push_back(std::dynamic_pointer_cast<EndeffectorSpline>(opt_vars->GetComponent(id)));
  }

  dim_ =  {X, Y, Z};
  SetRows(GetNumberOfNodes()*nom.GetCount()*dim_.size());
  converter_ = AngularStateConverter(base_angular_);
}

RangeOfMotionBox::~RangeOfMotionBox ()
{
}

int
RangeOfMotionBox::GetRow (int node, EndeffectorID ee, int dim) const
{
  return (node*nominal_stance_.GetCount() + ee) * dim_.size() + dim;
}

void
RangeOfMotionBox::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
  Vector3d base_W = base_linear_->GetPoint(t).p_;
  MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();

  for (auto ee : nominal_stance_.GetEEsOrdered()) {

    Vector3d pos_ee_B = b_R_w*(ee_splines_.at(ee)->GetPoint(t).p_ - base_W);

    for (auto dim : dim_)
      g(GetRow(k,ee,dim)) = pos_ee_B(dim);
  }
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k, VecBound& bounds) const
{
  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    for (auto dim : dim_) {
      Bound b;
      b += nominal_stance_.At(ee)(dim);
      b.upper_ += max_deviation_from_nominal_(dim);
      b.lower_ -= max_deviation_from_nominal_(dim);
      bounds.at(GetRow(k,ee,dim)) = b;
    }
  }
}

void
RangeOfMotionBox::UpdateJacobianAtInstance (double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  MatrixSXd b_R_w = converter_.GetRotationMatrixBaseToWorld(t).transpose();

  for (auto ee : nominal_stance_.GetEEsOrdered()) {

    int row_start = GetRow(k,ee,X);

    if (var_set == ee_splines_.at(ee)->GetName()) {
      jac.middleRows(row_start, kDim3d) = b_R_w*ee_splines_.at(ee)->GetJacobian(t,kPos);
    }

    if (var_set == base_linear_->GetName()) {
      jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobian(t, kPos);
    }

    if (var_set == base_angular_->GetName()) {
      Vector3d base_W   = base_linear_->GetPoint(t).p_;
      Vector3d ee_pos_W = ee_splines_.at(ee)->GetPoint(t).p_;
      Vector3d r_W = ee_pos_W - base_W;
      jac.middleRows(row_start, kDim3d) = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,r_W, true);
    }


  }
}

} /* namespace opt */
} /* namespace xpp */
