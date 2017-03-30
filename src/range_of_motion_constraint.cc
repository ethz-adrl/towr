/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint Classes
 */

#include <xpp/opt/constraints/range_of_motion_constraint.h>
#include <xpp/opt/base_motion.h>

namespace xpp {
namespace opt {

RangeOfMotionBox::RangeOfMotionBox (const ComMotionPtr& com_motion,
                                    const EEMotionPtr& ee_motion,
                                    double dt,
                                    const MaxDevXY& dev,
                                    const NominalStance& nom)
    :TimeDiscretizationConstraint(ee_motion->GetTotalTime(), dt)
{
  name_ = "Range of Motion";
  com_motion_       = com_motion;
  ee_motion_        = ee_motion;
  max_deviation_from_nominal_ = dev;
  nominal_stance_ = nom;

  // reserve space for every endeffector, but so far only fill constraints/bounds
  // for the ones in contacts, the others read 0 < g=0 < 0.
  int num_constraints = GetNumberOfNodes()*ee_motion_->GetNumberOfEndeffectors()*kDim2d;
  SetDependentVariables({com_motion, ee_motion}, num_constraints);
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
RangeOfMotionBox::UpdateConstraintAtInstance (double t, int k)
{
  Vector3d geom_W = com_motion_->GetBase(t).lin.p;

  for (const auto& c : ee_motion_->GetContacts(t)) {
    Vector3d pos_ee_B = c.p - geom_W;

    for (auto dim : {X,Y})
      g_(GetRow(k,c.ee,dim)) = pos_ee_B(dim);
  }
}

void
RangeOfMotionBox::UpdateBoundsAtInstance (double t, int k)
{
  for (auto c : ee_motion_->GetContacts(t)) {

    Vector3d f_nom_B = nominal_stance_.At(c.ee);
    for (auto dim : {X,Y}) {
      Bound b;
      b += f_nom_B(dim);
      b.upper_ += max_deviation_from_nominal_.at(dim);
      b.lower_ -= max_deviation_from_nominal_.at(dim);
      bounds_.at(GetRow(k,c.ee,dim)) = b;
    }
  }
}

void
RangeOfMotionBox::UpdateJacobianAtInstance (double t, int k)
{
  Jacobian& jac_ee   = GetJacobianRefWithRespectTo(ee_motion_->GetID());
  Jacobian& jac_base = GetJacobianRefWithRespectTo(com_motion_->GetID());

  for (auto c : ee_motion_->GetContacts(t)) {
    for (auto dim : d2::AllDimensions) {
      int row = GetRow(k,c.ee,dim);
      jac_ee.coeffRef(row, ee_motion_->Index(c.ee,c.id,dim)) = 1.0;
      jac_base.row(row) = -1*com_motion_->GetJacobian(t, kPos, static_cast<Coords3D>(dim));
    }
  }
}

} /* namespace opt */
} /* namespace xpp */
