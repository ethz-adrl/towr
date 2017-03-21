/**
 @file    range_of_motion_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares various Range of Motion Constraint Classes
 */

#include <xpp/opt/range_of_motion_constraint.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/variable_names.h>

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
RangeOfMotionConstraint::Init (const ComMotion& com_motion,
                               const EndeffectorsMotion& ee_motion,
                               double dt)
{
  com_motion_       = com_motion.clone();
  ee_motion_        = ee_motion;

  dts_.clear();
  double t = 0.0;
  for (int i=0; i<floor(ee_motion_.GetTotalTime()/dt); ++i) {
    dts_.push_back(t);
    t += dt;
  }
  dts_.push_back(ee_motion.GetTotalTime());
}

void
RangeOfMotionConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  com_motion_->SetCoefficients(x_coeff);

  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);
  ee_motion_.SetOptimizationParameters(footholds);

  // jacobians are constant, only need to be set once
  if (first_update_) {
    SetJacobianWrtContacts(jac_wrt_contacts_);
    SetJacobianWrtMotion(jac_wrt_motion_);
    first_update_ = false;
  }
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

RangeOfMotionBox::RangeOfMotionBox (const MaxDevXY& dev,
                                    const NominalStance& nom)
{
  max_deviation_from_nominal_ = dev;
  nominal_stance_ = nom;
}

RangeOfMotionBox::VectorXd
RangeOfMotionBox::EvaluateConstraint () const
{
  std::vector<double> g_vec;

  for (double t : dts_) {
//  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {
    PosXY geom_W = com_motion_->GetBase(t).lin.p.topRows<kDim2d>();


//    for (const auto& c : ee_motion_.GetEndeffectors(t).ToImpl()) {
    for (const auto& c : ee_motion_.GetContacts(t)) {
      // contact position expressed in base frame
      PosXY g;
      if (c.id == ContactBase::kFixedByStartStance)
        g = -geom_W;
      else
        g = c.p.topRows<kDim2d>() - geom_W;

      for (auto dim : {X,Y})
        g_vec.push_back(g(dim));

    }
  }

  return Eigen::Map<VectorXd>(&g_vec[0], g_vec.size());
}

VecBound
RangeOfMotionBox::GetBounds () const
{
  std::vector<Bound> bounds;
  for (double t : dts_) {
//  for (auto node : motion_structure_.GetPhaseStampedVec()) {
    for (auto c : ee_motion_.GetContacts(t)) {

      Vector3d f_nom_B = nominal_stance_.At(c.ee);
      for (auto dim : {X,Y}) {
        Bound b;
        b += f_nom_B(dim);
        b.upper_ += max_deviation_from_nominal_.at(dim);
        b.lower_ -= max_deviation_from_nominal_.at(dim);

        if (c.id == ContactBase::kFixedByStartStance)
          b -= c.p(dim);
//          for (auto f : node.contacts_fixed_)
//            if (f.ee == c.ee)
//              b -= f.p(dim);

        bounds.push_back(b);
      }
    }
  }
  return bounds;
}

void
RangeOfMotionBox::SetJacobianWrtContacts (Jacobian& jac_wrt_contacts) const
{
  int n_contacts = ee_motion_.GetAllFreeContacts().size() * kDim2d;//footholds_.size() * kDim2d;
  int m_constraints = GetNumberOfConstraints();
  jac_wrt_contacts = Jacobian(m_constraints, n_contacts);

  int row=0;
  for (double t : dts_) {
//  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {
    for (auto c : ee_motion_.GetContacts(t)) {
      if (c.id != ContactBase::kFixedByStartStance) {
        for (auto dim : {X,Y})
          jac_wrt_contacts.insert(row+dim, ee_motion_.Index(c.ee,c.id,dim)) = 1.0;
      }

      row += kDim2d;
    }
  }
}

void
RangeOfMotionBox::SetJacobianWrtMotion (Jacobian& jac_wrt_motion) const
{
  int n_motion   = com_motion_->GetTotalFreeCoeff();
  int m_constraints = GetNumberOfConstraints();
  jac_wrt_motion = Jacobian(m_constraints, n_motion);

  int row=0;
//  for (const auto& node : motion_structure_.GetPhaseStampedVec())
  for (double t : dts_)
    for (const auto c : ee_motion_.GetContacts(t))
      for (auto dim : {X,Y})
        jac_wrt_motion.row(row++) = -1*com_motion_->GetJacobian(t, kPos, dim);
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

