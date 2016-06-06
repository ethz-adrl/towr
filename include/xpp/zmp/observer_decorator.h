/**
 @file    observer_decorator.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSERVER_DECORATOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSERVER_DECORATOR_H_

#include <xpp/zmp/i_observer.h>

#include <memory>

namespace xpp {
namespace zmp {

class ObserverDecorator : public IObserver {
public:
  typedef std::unique_ptr<IObserver> ObserverPtr;

  ObserverDecorator (OptimizationVariables& subject, ObserverPtr& inner);
  virtual ~ObserverDecorator ();

  void Update() override;

private:
  ObserverPtr wrappee_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSERVER_DECORATOR_H_ */
