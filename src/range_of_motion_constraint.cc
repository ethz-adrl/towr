/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint Classes
 */

#include <xpp/opt/range_of_motion_constraint.h>
#include <xpp/opt/base_motion.h>

namespace xpp {
namespace opt {

RangeOfMotionConstraint::RangeOfMotionConstraint ()
{
  name_ = "Range of Motion";
}

RangeOfMotionConstraint::~RangeOfMotionConstraint ()
{
}

void
RangeOfMotionConstraint::Init (const ComMotionPtrU& com_motion,
                               const EEMotionPtr& ee_motion,
                               double dt)
{
  com_motion_       = com_motion;
  ee_motion_        = ee_motion;

  dts_.clear();
  double t = 0.0;
  for (int i=0; i<floor(ee_motion->GetTotalTime()/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }
  dts_.push_back(ee_motion->GetTotalTime()); // so final stance constrained as well


  int num_constraints = 0;
  for (double t : dts_) {
    int num_contacts = ee_motion_->GetContacts(t).size();
    num_constraints += kDim2d*num_contacts;
  }

  SetDependentVariables({com_motion, ee_motion}, num_constraints);
  InitializeConstantJacobians();
}

RangeOfMotionBox::RangeOfMotionBox (const MaxDevXY& dev,
                                    const NominalStance& nom)
{
  max_deviation_from_nominal_ = dev;
  nominal_stance_ = nom;
}

void
RangeOfMotionBox::UpdateConstraintValues ()
{
  int i = 0;
  for (double t : dts_) {
    PosXY geom_W = com_motion_->GetBase(t).lin.Get2D().p;

    for (const auto& c : ee_motion_->GetContacts(t)) {
      // contact position expressed in base frame
      PosXY pos_ee_B;
      if (c.id == ContactBase::kFixedByStartStance)
        pos_ee_B = -geom_W;
      else
        pos_ee_B = c.p.topRows<kDim2d>() - geom_W;

      for (auto dim : {X,Y})
        g_(i++) = pos_ee_B(dim);
    }
  }
}

VecBound
RangeOfMotionBox::GetBounds () const
{
  int i=0;
  for (double t : dts_) {
    for (auto c : ee_motion_->GetContacts(t)) {

      Vector3d f_nom_B = nominal_stance_.At(c.ee);
      for (auto dim : {X,Y}) {
        Bound b;
        b += f_nom_B(dim);
        b.upper_ += max_deviation_from_nominal_.at(dim);
        b.lower_ -= max_deviation_from_nominal_.at(dim);

        if (c.id == ContactBase::kFixedByStartStance)
          b -= c.p(dim);

        bounds_.at(i++) = b;
      }
    }
  }
  return bounds_;
}

void
RangeOfMotionBox::InitializeConstantJacobians ()
{
  UpdateJacobianWrtEndeffectors();
  UpdateJacobianWrtBase();
}

void
RangeOfMotionBox::UpdateJacobianWrtEndeffectors ()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(ee_motion_->GetID());

  int row=0;
  for (double t : dts_) {
    for (auto c : ee_motion_->GetContacts(t)) {
      if (c.id != ContactBase::kFixedByStartStance) {
        for (auto dim : d2::AllDimensions)
          jac.coeffRef(row+dim, ee_motion_->Index(c.ee,c.id,dim)) = 1.0;
      }

      row += kDim2d;
    }
  }
}

void
RangeOfMotionBox::UpdateJacobianWrtBase ()
{
  Jacobian& jac = GetJacobianRefWithRespectTo(com_motion_->GetID());

  int row=0;
  for (double t : dts_)
    for (const auto c : ee_motion_->GetContacts(t))
      for (auto dim : {X,Y})
        jac.row(row++) = -1*com_motion_->GetJacobian(t, kPos, dim);
}


//RangeOfMotionFixed::VectorXd
//RangeOfMotionFixed::EvaluateConstraint () const
//{
//  std::vector<double> g_vec;
//
//  auto feet = contacts_->GetFootholdsInWorld();
//  for (const auto& f : feet) {
//    g_vec.push_back(f.p.x());
//    g_vec.push_back(f.p.y());
//  }
//
//  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
//}
//
//RangeOfMotionFixed::VecBound
//RangeOfMotionFixed::GetBounds () const
//{
//  std::vector<Bound> bounds;
//
//  auto start_stance = contacts_->GetStartStance();
//  auto steps = contacts_->GetFootholdsInWorld();
//
//  for (const auto& s : steps) {
//    auto leg = s.leg;
//    auto start_foothold = hyq::Foothold::GetLastFoothold(leg, start_stance);
//
//    bounds.push_back(Bound(start_foothold.p.x() + kStepLength_,
//                           start_foothold.p.x() + kStepLength_));
//    bounds.push_back(Bound(start_foothold.p.y(), start_foothold.p.y()));
//  }
//
//  return bounds;
//}
//
//void
//RangeOfMotionFixed::SetJacobianWrtContacts (Jacobian& jac_wrt_contacts) const
//{
//  int n_contacts = contacts_->GetTotalFreeCoeff();
//  int m_constraints = n_contacts;
//  jac_wrt_contacts = Jacobian(m_constraints, n_contacts);
//  jac_wrt_contacts.setIdentity();
//}
//
//void
//RangeOfMotionFixed::SetJacobianWrtMotion (Jacobian& jac_wrt_motion) const
//{
//  // empty jacobian
//  int n_contacts = contacts_->GetTotalFreeCoeff();
//  int m_constraints = n_contacts;
//  jac_wrt_motion = Jacobian(m_constraints, n_contacts);
//}


} /* namespace opt */
} /* namespace xpp */

