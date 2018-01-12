/*
 * contact_schedule_observer.cc
 *
 *  Created on: Jan 10, 2018
 *      Author: winklera
 */

#include <towr/variables/contact_schedule_observer.h>

#include <towr/variables/contact_schedule.h>

namespace towr {


ContactScheduleObserver::ContactScheduleObserver (SubjectPtr subject)
{
  contact_schedule_ = subject;

  // register this observer to subject so this class always up-to-date
  subject->AddObserver(this);
}

} // namespace towr

