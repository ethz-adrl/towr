/**
 @file    interpreting_observer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Brief description
 */

#include <xpp/zmp/interpreting_observer.h>

#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

InterpretingObserver::InterpretingObserver (OptimizationVariables& subject)
    :IObserver(subject)
{
  interpreter_ = nullptr;
}

InterpretingObserver::~InterpretingObserver ()
{
  // TODO Auto-generated destructor stub
}

void
InterpretingObserver::SetInterpreter (const InterpreterPtr& interpreter)
{
  interpreter_ = interpreter; // now also interpreter_ has ownership of Interpreter-object owned by interpreter
}

void
InterpretingObserver::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
  x_feet_  = subject_->GetFootholdsStd();
}

InterpretingObserver::VecSpline
InterpretingObserver::GetSplines () const
{
  return interpreter_->GetSplines(x_coeff_);
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetFootholds () const
{
  return interpreter_->GetFootholds(x_feet_);
}

InterpretingObserver::VecFoothold
InterpretingObserver::GetStartStance () const
{
  return interpreter_->GetStartStance();
}

} /* namespace zmp */
} /* namespace xpp */
