/**
 @file    dynamic_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 4, 2016
 @brief   Defines the DynamicConstraint class
 */

#include <xpp/opt/dynamic_constraint.h>
#include <xpp/opt/com_motion.h>
#include <xpp/opt/optimization_variables.h>
#include <xpp/opt/zero_moment_point.h>

#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace opt {

DynamicConstraint::DynamicConstraint ()
{
  // TODO Auto-generated constructor stub
}

DynamicConstraint::~DynamicConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
DynamicConstraint::Init (const ComMotion& com_motion,
                         const Contacts& contacts,
                         const MotionStructure& motion_structure)
{
  com_motion_       = com_motion.clone();
  contacts_         = ContactPtrU(new Contacts(contacts));
  motion_structure_ = motion_structure;
}

void
DynamicConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x_coeff   = opt_var->GetVariables(VariableNames::kSplineCoeff);
  VectorXd footholds = opt_var->GetVariables(VariableNames::kFootholds);

  com_motion_->SetCoefficients(x_coeff);
  contacts_->SetFootholdsXY(utils::ConvertEigToStd(footholds));
}

DynamicConstraint::VectorXd
DynamicConstraint::EvaluateConstraint () const
{
  for (const auto& node : motion_structure_.GetPhaseStampedVec()) {
    auto com = com_motion_->GetCom(node.time_);


    static const double kWalkingHeight = 0.58; //zmp_ make parameter
    auto zmp = ZeroMomentPoint::CalcZmp(com.Make3D(), kWalkingHeight);






  }
}

DynamicConstraint::VecBound
DynamicConstraint::GetBounds () const
{
}

DynamicConstraint::Jacobian
DynamicConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
}

} /* namespace opt */
} /* namespace xpp */
