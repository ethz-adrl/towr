/*
 * contact_schedule_observer.h
 *
 *  Created on: Jan 9, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_CONTACT_SCHEDULE_OBSERVER_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_CONTACT_SCHEDULE_OBSERVER_H_

#include <memory>

namespace towr {


class ContactSchedule;


class ContactScheduleObserver {
public:
  using SubjectPtr = ContactSchedule*; // observer shouldn't own subject

  ContactScheduleObserver() = default;
  ContactScheduleObserver(SubjectPtr contact_schedule);

  virtual ~ContactScheduleObserver() = default;

  virtual void UpdatePhaseDurations() = 0;

protected:
  SubjectPtr contact_schedule_;
};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_CONTACT_SCHEDULE_OBSERVER_H_ */
