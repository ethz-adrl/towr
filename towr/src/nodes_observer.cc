/*
 * node_observer.cc
 *
 *  Created on: Jan 10, 2018
 *      Author: winklera
 */

#include <towr/variables/nodes_observer.h>

#include <towr/variables/node_values.h>

namespace towr {

NodesObserver::NodesObserver(SubjectPtr subject)
{
  node_values_ = subject;

  // register observer to subject so this class always up-to-date
  subject->AddObserver(this);
};


} // namespace towr


