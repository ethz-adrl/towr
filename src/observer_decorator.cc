/**
 @file    observer_decorator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/zmp/observer_decorator.h>

namespace xpp {
namespace zmp {

ObserverDecorator::ObserverDecorator (OptimizationVariables& subject,
                                      ObserverPtr& inner)
    :IObserver(subject)
{
  wrappee_ = std::move(inner); // now wrappee_ is owner of inner observer object
}

ObserverDecorator::~ObserverDecorator ()
{
  // TODO Auto-generated destructor stub
}

void
ObserverDecorator::Update ()
{
  wrappee_->Update();
}

} /* namespace zmp */
} /* namespace xpp */
