/*
 * i_observer.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include "../include/xpp/observer.h"

#include <xpp/optimization_variables.h>

namespace xpp {
namespace opt {

Observer::Observer (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

Observer::~Observer ()
{
  subject_->DeregisterObserver(this);
}

} /* namespace zmp */
} /* namespace xpp */
