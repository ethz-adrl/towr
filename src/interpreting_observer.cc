/**
 @file    interpreting_observer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Brief description
 */

#include <xpp/zmp/interpreting_observer.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd VectorXd;

InterpretingObserver::InterpretingObserver (OptimizationVariables& subject)
    :IObserver(subject)
{
}

InterpretingObserver::~InterpretingObserver ()
{
  // TODO Auto-generated destructor stub
}

void
InterpretingObserver::SetInterpreter (const Interpreter& interpreter)
{
  interpreter_ = interpreter;
}

void
InterpretingObserver::Update ()
{
  VectorXd x_coeff_    = subject_->GetVariables(VariableNames::kSplineCoeff);
  VectorXd x_footholds = subject_->GetVariables(VariableNames::kFootholds);

  // smell this could be a cause for slow performance
  interpreter_.SetFootholds(utils::ConvertEigToStd(x_footholds));
  interpreter_.SetSplineCoefficients(x_coeff_);
}

InterpretingObserver::VecSpline
InterpretingObserver::GetSplines () const
{
  return interpreter_.GetSplines();
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetFootholds () const
{
  return interpreter_.GetFootholds();
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetStartStance () const
{
  return interpreter_.GetStartStance();
}

} /* namespace zmp */
} /* namespace xpp */
