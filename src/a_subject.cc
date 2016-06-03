/*
 * a_subject.cc
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#include <xpp/zmp/a_subject.h>

namespace xpp {
namespace zmp {

ASubject::ASubject ()
{
  // TODO Auto-generated constructor stub
}

ASubject::~ASubject ()
{
  // TODO Auto-generated destructor stub
}

int
ASubject::GetObserverCount () const
{
  return observers_.size();
}

void
ASubject::RemoveObservers ()
{
  observers_.clear();
}

void
ASubject::RegisterObserver(IObserver* o)
{
  bool observer_already_registered = std::find(observers_.begin(), observers_.end(), o) != observers_.end();
  if (!observer_already_registered)
    observers_.push_back(o);
}

void
ASubject::NotifyObservers () const
{
  for (IObserver* const o : observers_)
    o->Update();
}

} /* namespace zmp */
} /* namespace xpp */

