/**
 @file    a_subject.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides an abstract base class to implement the Subject in the \
          observer pattern (https://sourcemaking.com/design_patterns/observer/cpp/3)
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SUBJECT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SUBJECT_H_

#include "observer.h"

namespace xpp {
namespace opt {

/** Holds and distributes information to the connected observers.
  *
  * See observer pattern:
  * https://sourcemaking.com/design_patterns/observer/cpp/3
  */
class Subject {
public:
  Subject ();
  virtual ~Subject ();

  void RegisterObserver(Observer*);
  void DeregisterObserver(Observer* o);
  int GetObserverCount() const;

protected:
  void NotifyObservers() const;

private:
  std::vector<Observer*> observers_;
  void RemoveObservers();

  // delete the copy and copy assignment operators, since that messes up the
  // observer pattern logic
  Subject(Subject const&) = delete;
  void operator=(Subject const&)        = delete;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_A_SUBJECT_H_ */
