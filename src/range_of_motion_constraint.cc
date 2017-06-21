/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Defines the RangeOfMotionBox class.
 */

#include <xpp/opt/constraints/range_of_motion_constraint.h>

#include <string>
#include <vector>
#include <Eigen/Dense>

#include <xpp/cartesian_declarations.h>
#include <xpp/state.h>

#include <xpp/bound.h>
#include <xpp/opt/variables/base_motion.h>
#include <xpp/opt/variables/endeffectors_motion.h>

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

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>        (opt_vars->GetComponent("base_motion"));
  ee_motion_  = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetComponent("endeffectors_motion"));

  dim_ =  {X, Y, Z};
  SetRows(GetNumberOfNodes()*nom.GetCount()*dim_.size());
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
  Vector3d base_W = com_motion_->GetBase(t).lin.p_;

  auto pos_ee_W = ee_motion_->GetEndeffectors(t);

  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    // zmp_ for now don't take into account lifting the leg
    // because i don't have a jacobian for the swingleg motion yet?
    pos_ee_W.At(ee).p_.z() = 0.0;

    Vector3d pos_ee_B = pos_ee_W.At(ee).p_ - base_W;

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
  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    for (auto dim : dim_) {
      int row = GetRow(k,ee,dim);

      if (var_set == ee_motion_->GetName()) {
        if (dim == X || dim == Y) {
          auto dim_d2 = static_cast<d2::Coords>(dim); // because don't have jacobian of lifting leg
          jac.row(row) = ee_motion_->GetJacobianPos(t,ee,dim_d2);
        }
      }

      if (var_set == com_motion_->GetName())
        jac.row(row) = -1*com_motion_->GetJacobian(t, kPos, To6D(dim));
    }
  }
}

} /* namespace opt */
} /* namespace xpp */
