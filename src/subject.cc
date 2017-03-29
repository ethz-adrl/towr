/*
 * a_subject.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include "../include/xpp/subject.h"

#include <iostream>

namespace xpp {
namespace opt {

Subject::Subject ()
{
  // TODO Auto-generated constructor stub
}

Subject::~Subject ()
{
  // TODO Auto-generated destructor stub
}

int
Subject::GetObserverCount () const
{
  return observers_.size();
}

void
Subject::RemoveObservers ()
{
  observers_.clear();
}

void
Subject::RegisterObserver(Observer* o)
{
  bool observer_already_registered = std::find(observers_.begin(), observers_.end(), o) != observers_.end();
  if (!observer_already_registered)
    observers_.push_back(o);
}

void
Subject::DeregisterObserver(Observer* o)
{
  auto it = std::find(observers_.begin(), observers_.end(), o);
  observers_.erase(it);
}

void
Subject::NotifyObservers () const
{
  for (Observer* const o : observers_)
    o->Update();
}

} /* namespace zmp */
} /* namespace xpp */

