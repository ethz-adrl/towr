/**
 @file    interpreting_observer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Brief description
 */

#include <xpp/zmp/interpreting_observer.h>
#include <xpp/zmp/com_motion.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd VectorXd;

InterpretingObserver::InterpretingObserver (OptimizationVariables& subject)
    :IObserver(subject)
{
  com_motion_   = nullptr;
  contacts_     = nullptr;
}

InterpretingObserver::~InterpretingObserver ()
{
  // TODO Auto-generated destructor stub
}

void
InterpretingObserver::Init(const MotionStructure& structure,
                           const ComMotion& com_motion,
                           const Contacts& contacts)
{
  com_motion_       = com_motion.clone();
  contacts_         = ContactsPtrU(new Contacts(contacts));
  motion_structure_ = structure;
}

void
InterpretingObserver::Update ()
{
  VectorXd x_motion    = subject_->GetVariables(VariableNames::kSplineCoeff);
  VectorXd x_contacts  = subject_->GetVariables(VariableNames::kFootholds);

  com_motion_->SetCoefficients(x_motion);
  contacts_->SetFootholdsXY(utils::ConvertEigToStd(x_contacts));
}

InterpretingObserver::MotionPtrS
InterpretingObserver::GetComMotion() const
{
  return com_motion_;
};

MotionStructure
InterpretingObserver::GetStructure() const
{
  return motion_structure_;
};

InterpretingObserver::VecFoothold
InterpretingObserver::GetFootholds () const
{
  return contacts_->GetFootholds();
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetStartStance () const
{
  return contacts_->GetStartStance();
}

} /* namespace zmp */
} /* namespace xpp */
