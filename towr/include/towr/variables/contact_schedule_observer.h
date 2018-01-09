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

class ContactScheduleObserver {
public:
  using Ptr = std::shared_ptr<ContactScheduleObserver>;

  ContactScheduleObserver() = default;
  virtual ~ContactScheduleObserver() = default;

  virtual void UpdatePhaseDurations() = 0;

};

} /* namespace towr */

#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_CONTACT_SCHEDULE_OBSERVER_H_ */
