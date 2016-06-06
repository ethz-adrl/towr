/*
 * i_observer.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

IObserver::IObserver (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

IObserver::~IObserver ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace zmp */
} /* namespace xpp */
