/**
 @file    motion_structure.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint Classes
 */

#include <xpp/zmp/range_of_motion_constraint.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/com_motion.h>
#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

RangeOfMotionConstraint::RangeOfMotionConstraint ()
{
}

void
RangeOfMotionConstraint::Init (const ComMotion& com_motion, const Contacts& contacts)
{
  com_motion_ = com_motion.clone();
  contacts_   = ContactPtrU(new Contacts(contacts));

  auto start_feet = contacts_->GetStartStance();
  MotionStructure::LegIDVec start_legs;
  for (const auto& f : start_feet) {
    start_legs.push_back(f.leg);
  }

  auto step_feet = contacts_->GetFootholds();
  MotionStructure::LegIDVec step_legs;
  for (const auto& f : step_feet) {
    step_legs.push_back(f.leg);
  }

  double dt = 0.1;   // the times at which to evalute the constraint
  motion_structure_.Init(start_legs, step_legs, com_motion_->GetPhases(), dt);

  SetJacobianWrtContacts(jac_wrt_contacts_);
  SetJacobianWrtMotion(jac_wrt_motion_);
}

void
RangeOfMotionConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);

  com_motion_->SetCoefficients(x_coeff);
  contacts_->SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

RangeOfMotionConstraint::Jacobian
RangeOfMotionConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  if (var_set == VariableNames::kFootholds)
    return jac_wrt_contacts_;
  else if (var_set == VariableNames::kSplineCoeff)
    return jac_wrt_motion_;
  else
    return Jacobian();
}

RangeOfMotionBox::VectorXd
RangeOfMotionBox::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  for (const auto& node : motion_structure_.GetContactInfoVec()) {
    PosXY com_pos = com_motion_->GetCom(node.time_).p;

    for (const auto& f_id : node.foothold_ids_) {

      PosXY contact_pos = PosXY::Zero();

      if(f_id != xpp::hyq::Foothold::kFixedByStart) {
        auto footholds = contacts_->GetFootholds();
        contact_pos = footholds.at(f_id).p.topRows(kDim2d);
      }

      for (auto dim : {X,Y})
        g_vec.push_back(contact_pos(dim) - com_pos(dim));

    }
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionBox::VecBound
RangeOfMotionBox::GetBounds () const
{
  std::vector<Bound> bounds;
  for (auto node : motion_structure_.GetContactInfoVec()) {

    int c=0;
    for (auto leg : node.legs_) {

      PosXY start_offset = PosXY::Zero();

      if (node.foothold_ids_.at(c) == xpp::hyq::Foothold::kFixedByStart)
        start_offset = contacts_->GetStartFoothold(leg).p.topRows(kDim2d);

      PosXY pos_nom_B = contacts_->GetNominalPositionInBase(leg);
      for (auto dim : {X,Y}) {
        Bound b;
        b.upper_ = pos_nom_B(dim) + kBoxLength_/2.;
        b.lower_ = pos_nom_B(dim) - kBoxLength_/2.;
        b -= start_offset(dim);
        bounds.push_back(b);
      }

      c++;
    }
  }
  return bounds;
}

void
RangeOfMotionBox::SetJacobianWrtContacts (Jacobian& jac_wrt_contacts) const
{
  int n_contacts = contacts_->GetTotalFreeCoeff();
  int m_constraints = motion_structure_.GetTotalNumberOfDiscreteContacts() * kDim2d;
  jac_wrt_contacts = Jacobian(m_constraints, n_contacts);

  int row=0;
  for (const auto& node : motion_structure_.GetContactInfoVec()) {

    for (auto f_id : node.foothold_ids_) {

      if (f_id != xpp::hyq::Foothold::kFixedByStart) {
        for (auto dim : {X,Y})
          jac_wrt_contacts.insert(row+dim, Contacts::Index(f_id,dim)) = 1.0;
      }

      row += kDim2d;
    }
  }
}

void
RangeOfMotionBox::SetJacobianWrtMotion (Jacobian& jac_wrt_motion) const
{
  int n_motion   = com_motion_->GetTotalFreeCoeff();
  int m_constraints = motion_structure_.GetTotalNumberOfDiscreteContacts() * kDim2d;
  jac_wrt_motion = Jacobian(m_constraints, n_motion);

  int row=0;
  for (const auto& node : motion_structure_.GetContactInfoVec())
    for (const auto contact : node.foothold_ids_)
      for (auto dim : {X,Y})
        jac_wrt_motion.row(row++) = -1*com_motion_->GetJacobian(node.time_, kPos, dim);
}


RangeOfMotionFixed::VectorXd
RangeOfMotionFixed::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  auto feet = contacts_->GetFootholds();
  for (const auto& f : feet) {
    g_vec.push_back(f.p.x());
    g_vec.push_back(f.p.y());
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionFixed::VecBound
RangeOfMotionFixed::GetBounds () const
{
  std::vector<Bound> bounds;

  auto start_stance = contacts_->GetStartStance();
  auto steps = contacts_->GetFootholds();

  for (const auto& s : steps) {
    auto leg = s.leg;
    auto start_foothold = hyq::Foothold::GetLastFoothold(leg, start_stance);

    bounds.push_back(Bound(start_foothold.p.x() + kStepLength_,
                           start_foothold.p.x() + kStepLength_));
    bounds.push_back(Bound(start_foothold.p.y(), start_foothold.p.y()));
  }

  return bounds;
}

void
RangeOfMotionFixed::SetJacobianWrtContacts (Jacobian& jac_wrt_contacts) const
{
  int n_contacts = contacts_->GetTotalFreeCoeff();
  int m_constraints = n_contacts;
  jac_wrt_contacts = Jacobian(m_constraints, n_contacts);
  jac_wrt_contacts.setIdentity();
}

void
RangeOfMotionFixed::SetJacobianWrtMotion (Jacobian& jac_wrt_motion) const
{
  // empty jacobian
  int n_contacts = contacts_->GetTotalFreeCoeff();
  int m_constraints = n_contacts;
  jac_wrt_motion = Jacobian(m_constraints, n_contacts);
}


} /* namespace zmp */
} /* namespace xpp */

