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
#include <xpp/zmp/a_robot_interface.h>

namespace xpp {
namespace zmp {

using namespace xpp::utils::coords_wrapper;

RangeOfMotionConstraint::RangeOfMotionConstraint ()
{
}

RangeOfMotionConstraint::~RangeOfMotionConstraint ()
{
}

void
RangeOfMotionConstraint::Init (const ComMotion& com_motion,
                               const Contacts& contacts,
                               const MotionStructure& motion_structure,
                               RobotPtrU p_robot)
{
  com_motion_       = com_motion.clone();
  contacts_         = ContactPtrU(new Contacts(contacts));
  motion_structure_ = motion_structure;
  robot_            = std::move(p_robot);

  motion_structure_.SetDisretization(0.3);

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

bool
RangeOfMotionBox::IsPositionInsideRangeOfMotion (
    const PosXY& pos, const Stance& stance, const ARobotInterface& robot)
{
  double max_deviation = robot.GetMaxDeviationXYFromNominal();

  for (auto f : stance) {
    auto p_nominal = robot.GetNominalStanceInBase(f.leg);

    for (auto dim : {X,Y}) {

      double distance_to_foot = f.p(dim) - pos(dim);
      double distance_to_nom  = distance_to_foot - p_nominal(dim);

      if (std::abs(distance_to_nom) > max_deviation)
        return false;
    }
  }

  return true;
}

RangeOfMotionBox::VectorXd
RangeOfMotionBox::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  for (const auto& node : motion_structure_.GetContactInfoVec()) {
    PosXY com_pos = com_motion_->GetCom(node.time_).p;

    for (const auto& c : node.phase_.free_contacts_) {

      PosXY contact_pos = PosXY::Zero();

      if(c.id != xpp::hyq::Foothold::kFixedByStart) {
        auto footholds = contacts_->GetFootholds();
        contact_pos = footholds.at(c.id).p.topRows(kDim2d);
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
  const double max_deviation = robot_->GetMaxDeviationXYFromNominal();

  std::vector<Bound> bounds;
  for (auto node : motion_structure_.GetContactInfoVec()) {

    for (auto c : node.phase_.free_contacts_) {

      PosXY start_offset = PosXY::Zero();

      if (c.id == xpp::hyq::Foothold::kFixedByStart)
        start_offset = contacts_->GetStartFoothold(static_cast<xpp::hyq::LegID>(c.ee)).p.topRows(kDim2d);

      PosXY pos_nom_B = robot_->GetNominalStanceInBase(static_cast<xpp::hyq::LegID>(c.ee));
      for (auto dim : {X,Y}) {
        Bound b;
        b.upper_ = pos_nom_B(dim) + max_deviation;
        b.lower_ = pos_nom_B(dim) - max_deviation;
        b -= start_offset(dim);
        bounds.push_back(b);
      }
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
  for (const auto& node : motion_structure_.GetContactInfoVec())
    for (auto c : node.phase_.free_contacts_) {
      if (c.id != xpp::hyq::Foothold::kFixedByStart)
        for (auto dim : {X,Y})
          jac_wrt_contacts.insert(row+dim, Contacts::Index(c.id,dim)) = 1.0;

      row += kDim2d;
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
    for (const auto c : node.phase_.free_contacts_)
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

