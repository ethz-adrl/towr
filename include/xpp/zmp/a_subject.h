/*
 * a_subject.h
 *
 *  Created on: May 24, 2016
 *      Author: winklera
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SUBJECT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SUBJECT_H_

#include <xpp/zmp/i_observer.h>

namespace xpp {
namespace zmp {

class ASubject {
public:
  ASubject ();
  virtual ~ASubject ();

  int GetObserverCount() const;

protected:
  std::vector<IObserver*> observers_;

private:
  // these methods never need to be called from the base class or base class pointer anyway
  virtual void RegisterObserver(IObserver*) = 0;
  virtual void NotifyObservers() const = 0;
//  virtual void RemoveObserver();
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SUBJECT_H_ */
