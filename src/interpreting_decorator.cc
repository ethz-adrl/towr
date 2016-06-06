/**
 @file    a_interpreting_observer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/interpreting_decorator.h>

#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

InterpretingDecorator::InterpretingDecorator (OptimizationVariables& subject,
                                              ObserverPtr& core)
    :ObserverDecorator(subject, core)
{
  // TODO Auto-generated constructor stub
}

InterpretingDecorator::~InterpretingDecorator ()
{
  // TODO Auto-generated destructor stub
}

void
InterpretingDecorator::Update ()
{
  ObserverDecorator::Update(); // functionality of the wrappee inner class

  // added functionality
  splines_   = interpreter_.GetSplines(subject_->GetSplineCoefficients());
  footholds_ = interpreter_.GetFootholds(subject_->GetFootholdsStd());
}

} /* namespace zmp */
} /* namespace xpp */
