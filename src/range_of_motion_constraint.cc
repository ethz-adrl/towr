/*
 * range_of_motion_constraint.cc
 *
 *  Created on: May 26, 2016
 *      Author: winklera
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


  // the times at which to evalute the constraint
  double dt = 0.1;

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

  MotionStructure motion_structure;
  motion_structure.Init(start_legs, step_legs, com_motion_->GetPhases());
  motion_info_ = motion_structure.GetContactInfoVec(dt);


  int n_contacts = contacts_->GetTotalFreeCoeff();
  int n_motion   = com_motion_->GetTotalFreeCoeff();
  int m_constraints = motion_info_.size() * kDim2d;

  jac_wrt_contacts_ = Jacobian(m_constraints, n_contacts);
  jac_wrt_motion_   = Jacobian(m_constraints, n_motion);

  SetJacobianWrtContacts();
  SetJacobianWrtMotion();
}

void
RangeOfMotionConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);

  com_motion_->SetCoefficients(x_coeff);
  contacts_->SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

RangeOfMotionConstraint::VectorXd
RangeOfMotionConstraint::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  for (const auto& c : motion_info_) {

    PosXY com_pos = com_motion_->GetCom(c.time_).p;

    PosXY contact_pos = PosXY::Zero();

    if(c.foothold_id_ != xpp::hyq::Foothold::kFixedByStart) {
      auto footholds = contacts_->GetFootholds();
      contact_pos = footholds.at(c.foothold_id_).p.topRows(kDim2d);
    }

    for (auto dim : {X,Y})
      g_vec.push_back(contact_pos(dim) - com_pos(dim));

  }

//  // refactor _write out really simple constraint just to test ZMP motion
//  auto feet = supp_polygon_container_->GetFootholds();
//  for (const auto& f : feet) {
//    g_vec.push_back(f.p.x());
//    g_vec.push_back(f.p.y());
//  }


  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

RangeOfMotionConstraint::VecBound
RangeOfMotionConstraint::GetBounds () const
{
  std::vector<Bound> bounds;

  double d = 0.3; // bounding box edge length of each foot
  for (auto c :  motion_info_) {

    PosXY start_offset = PosXY::Zero(); // because initial foothold is fixed
    if (c.foothold_id_ == xpp::hyq::Foothold::kFixedByStart) {
      start_offset = contacts_->GetStartFoothold(c.leg_).p.topRows(kDim2d);
    }

    PosXY pos_nom_B = GetNominalPositionInBase(c.leg_);
    for (auto dim : {X,Y}) {
      Bound b;
      b.upper_ = pos_nom_B(dim) + d/2.;
      b.lower_ = pos_nom_B(dim) - d/2.;
      b -= start_offset(dim);
      bounds.push_back(b);
    }

  }

//  // this is for creating fixed footholds (remember to comment in constraints above as well)
//  auto start_stance = supp_polygon_container_->GetStartStance();
//  auto steps = supp_polygon_container_->GetFootholds();
//
//  double step_length = 0.15;
//  for (const auto& s : steps) {
//    auto leg = s.leg;
//    auto start_foothold = hyq::Foothold::GetLastFoothold(leg, start_stance);
//
//    bounds.push_back(Bound(start_foothold.p.x() + step_length, start_foothold.p.x() + step_length));
//    bounds.push_back(Bound(start_foothold.p.y(), start_foothold.p.y()));
//  }

  return bounds;
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

//  // keep this somehow, nice for debugging
//  if (var_set == VariableNames::kFootholds) {
//    int n = supp_polygon_container_->GetTotalFreeCoeff();
//    jac = Jacobian(n,n);
//    jac.setIdentity();
//  }
}

RangeOfMotionConstraint::PosXY
RangeOfMotionConstraint::GetNominalPositionInBase (LegID leg) const
{
  const double x_nominal_b = 0.36; // 0.4
  const double y_nominal_b = 0.33; // 0.4

  switch (leg) {
    case hyq::LF: return PosXY( x_nominal_b,   y_nominal_b); break;
    case hyq::RF: return PosXY( x_nominal_b,  -y_nominal_b); break;
    case hyq::LH: return PosXY(-x_nominal_b,   y_nominal_b); break;
    case hyq::RH: return PosXY(-x_nominal_b,  -y_nominal_b); break;
    default: assert(false); // this should never happen
  }
}

void
RangeOfMotionConstraint::SetJacobianWrtContacts ()
{
  int row=0;
  for (const auto& c : motion_info_) {

    int id = c.foothold_id_;
    if (id != xpp::hyq::Foothold::kFixedByStart)
      for (auto dim : {X,Y})
        jac_wrt_contacts_.insert(row+dim, Contacts::Index(id,dim)) = 1.0;

    row += kDim2d;
  }
}

void
RangeOfMotionConstraint::SetJacobianWrtMotion ()
{
  int row=0;
  for (const auto& c : motion_info_)
    for (auto dim : {X,Y})
      jac_wrt_motion_.row(row++) = -com_motion_->GetJacobian(c.time_, kPos, dim);
}

} /* namespace zmp */
} /* namespace xpp */

