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
    :TimeDiscretizationConstraint(T, dt, nom.GetCount()*kDim2d)
{
//  name_ = "Range of Motion";
  max_deviation_from_nominal_ = dev;
  nominal_stance_ = nom;

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>        (opt_vars->GetSet("base_motion"));
  ee_motion_  = std::dynamic_pointer_cast<EndeffectorsMotion>(opt_vars->GetSet("endeffectors_motion"));

  // zmp_ DRY with number of constraints in base class constructor
  int num_constraints = GetNumberOfNodes()*ee_motion_->GetNumberOfEndeffectors()*kDim2d;
  SetDimensions(opt_vars, num_constraints);
}

RangeOfMotionBox::~RangeOfMotionBox ()
{
}

int
RangeOfMotionBox::GetRow (int node, EndeffectorID ee, int dim) const
{
  return (node*ee_motion_->GetNumberOfEndeffectors() + ee) * kDim2d + dim;
}

void
RangeOfMotionBox::UpdateConstraintAtInstance (double t, int k) const
{
  Vector3d base_W = com_motion_->GetBase(t).lin.p;

  auto pos_ee_W = ee_motion_->GetEndeffectors(t);

  for (auto ee : pos_ee_W.GetEEsOrdered()) {
    Vector3d pos_ee_B = pos_ee_W.At(ee).p - base_W;

    for (auto dim : {X,Y})
      g_new_(GetRow(k,ee,dim)) = pos_ee_B(dim);
  }
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k) const
{
  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    for (auto dim : d2::AllDimensions) {
      Bound b;
      b += nominal_stance_.At(ee)(dim);
      b.upper_ += max_deviation_from_nominal_.at(dim);
      b.lower_ -= max_deviation_from_nominal_.at(dim);
      bounds_.at(GetRow(k,ee,dim)) = b;
    }
  }
}

void
RangeOfMotionBox::UpdateJacobianAtInstance (double t, int k,
                                            Jacobian& jac, std::string var_set) const
{
  for (auto ee : nominal_stance_.GetEEsOrdered()) {
    for (auto dim : d2::AllDimensions) {
      int row = GetRow(k,ee,dim);

      if (var_set == ee_motion_->GetId())
        jac.row(row) = ee_motion_->GetJacobianWrtOptParams(t,ee,dim);

      if (var_set == com_motion_->GetId())
        jac.row(row) = -1*com_motion_->GetJacobian(t, kPos, static_cast<Coords3D>(dim));
    }
  }
}

} /* namespace opt */
} /* namespace xpp */
