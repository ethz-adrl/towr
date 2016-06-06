/**
 @file    a_interpreting_observer.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/a_interpreting_observer.h>

namespace xpp {
namespace zmp {

AInterpretingObserver::AInterpretingObserver (OptimizationVariables& subject)
    :IObserver(subject)
{
  // TODO Auto-generated constructor stub
}

AInterpretingObserver::~AInterpretingObserver ()
{
  // TODO Auto-generated destructor stub
}

void
AInterpretingObserver::Update ()
{
//  splines_   = interpreter_.GetSplines(subject_->GetSplineCoefficients());
//  footholds_ = interpreter_.GetFootholds(subject_->GetFootholdsStd());

}

} /* namespace zmp */
} /* namespace xpp */
