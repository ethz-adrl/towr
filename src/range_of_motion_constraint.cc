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
#include <xpp/state.h>

#include <xpp/bound.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial_spline.h>
#include <xpp/opt/variables/endeffectors_motion.h>
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
  ee_motion_    = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetComponent(id::endeffectors_motion));

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
  return (node*ee_motion_->GetNumberOfEndeffectors() + ee) * dim_.size() + dim;
}

void
RangeOfMotionBox::UpdateConstraintAtInstance (double t, int k, VectorXd& g) const
{
  Vector3d base_W = base_linear_->GetPoint(t).p_;
  MatrixSXd b_R_w  = converter_.GetRotationMatrix(t);

  auto pos_ee_W = ee_motion_->GetEndeffectors(t);

  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    // zmp_ for now don't take into account lifting the leg
    // because i don't have a jacobian for the swingleg motion yet?
    pos_ee_W.At(ee).p_.z() = 0.0;

    Vector3d pos_ee_B = b_R_w*(pos_ee_W.At(ee).p_ - base_W);

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
      b.upper_ += max_deviation_from_nominal_.at(dim);
      b.lower_ -= max_deviation_from_nominal_.at(dim);
      bounds.at(GetRow(k,ee,dim)) = b;
    }
  }
}

void
RangeOfMotionBox::UpdateJacobianAtInstance (double t, int k, Jacobian& jac,
                                            std::string var_set) const
{
  MatrixSXd b_R_w = converter_.GetRotationMatrix(t);

  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    int row_start = GetRow(k,ee,X);


    if (var_set == ee_motion_->GetName()) {
      // zmp_ move somewhere else
      // also, add jacobian of y
      Jacobian jac_ee_pos(kDim3d, ee_motion_->GetRows());
      jac_ee_pos.row(X) = ee_motion_->GetJacobianPos(t,ee,d2::X);
      jac_ee_pos.row(Y) = ee_motion_->GetJacobianPos(t,ee,d2::Y);

      jac.middleRows(row_start, kDim3d) = b_R_w*jac_ee_pos;
    }


    if (var_set == base_linear_->GetName()) {
      jac.middleRows(row_start, kDim3d) = -1*b_R_w*base_linear_->GetJacobian(t, kPos);
    }


    if (var_set == base_angular_->GetName()) {

      Vector3d base_W   = base_linear_->GetPoint(t).p_;
      Vector3d ee_pos_W = ee_motion_->GetEndeffectors(t).At(ee).p_;
      MatrixSXd base_to_ee_W = (ee_pos_W - base_W).sparseView(1.0, -1.0);

      for (auto dim : {X,Y,Z}) {
        Jacobian jac_row = converter_.GetDerivativeOfRotationMatrixRowWrtCoeff(t,dim);
        jac.row(GetRow(k,ee,dim)) = base_to_ee_W.transpose() * jac_row;
      }

    }


  }
}

} /* namespace opt */
} /* namespace xpp */
