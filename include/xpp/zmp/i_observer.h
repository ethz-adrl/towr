/*
 * i_observer.h
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_OBSERVER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_OBSERVER_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {

class IObserver {
public:
  IObserver ();
  virtual ~IObserver ();

  virtual void Update() = 0;

private:
  // delete the copy and copy assignment operators, since that messes up the
  // logic with the subject in the observer pattern.
  IObserver& operator=(const IObserver&) = delete;
  IObserver(const IObserver&)            = delete;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_OBSERVER_H_ */
