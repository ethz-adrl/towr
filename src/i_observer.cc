/*
 * i_observer.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/i_observer.h>
#include <xpp/optimization_variables.h>

namespace xpp {
namespace opt {

IObserver::IObserver (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

IObserver::~IObserver ()
{
  subject_->DeregisterObserver(this);
}

} /* namespace zmp */
} /* namespace xpp */
