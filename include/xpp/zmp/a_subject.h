/**
 @file    a_subject.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides an abstract base class to implement the Subject in the \
          observer pattern (https://sourcemaking.com/design_patterns/observer/cpp/3)
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
  void RemoveObservers();
  void RegisterObserver(IObserver*);

  // fixme think about not updating all observers (costs, constraints) all the time
  void NotifyObservers() const;

protected:
  std::vector<IObserver*> observers_;

private:
  // delete the copy and copy assignment operators, since that messes up the
  // observer pattern logic
  ASubject(ASubject const&) = delete;
  void operator=(ASubject const&)        = delete;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_SUBJECT_H_ */
