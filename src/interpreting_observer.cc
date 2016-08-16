/**
 @file    interpreting_observer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Brief description
 */

#include <xpp/zmp/interpreting_observer.h>

namespace xpp {
namespace zmp {

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
  x_coeff_                  = subject_->GetVariables(OptimizationVariables::kSplineCoeff);
  Eigen::VectorXd footholds = subject_->GetVariables(OptimizationVariables::kFootholds);

  x_feet_    = utils::ConvertEigToStd(footholds);
}

InterpretingObserver::VecSpline
InterpretingObserver::GetSplines () const
{
  return interpreter_.GetSplines(x_coeff_);
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetFootholds () const
{
  return interpreter_.GetFootholds(x_feet_);
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetStartStance () const
{
  return interpreter_.GetStartStance();
}

} /* namespace zmp */
} /* namespace xpp */
