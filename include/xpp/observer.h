/**
 @file    observer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides an abstract base class to implement the Observer in the
          observer pattern.
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_OBSERVER_H_
#define XPP_OPT_INCLUDE_XPP_OPT_OBSERVER_H_

#include <Eigen/Dense>

namespace xpp {
namespace opt {

// zmp_ ! this is also unclean, that general observer knows about
// optimization variables...
class OptimizationVariables; // forward decl. to avoid circular dependencies

/** Subscribes to the Subject to get up-to-date optimization variables.
 *
  * See observer pattern:
  * https://sourcemaking.com/design_patterns/observer/cpp/3
  */
class Observer {
public:
  Observer (OptimizationVariables& subject);
  virtual ~Observer ();

  virtual void Update() = 0;

protected:
  /** this variable holds the current state of optimization variables */
  OptimizationVariables* subject_;

private:
  // delete the copy and copy assignment operators, since that messes up the
  // logic with the subject in the observer pattern.
  Observer& operator=(const Observer&) = delete;
  Observer(const Observer&)            = delete;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_OBSERVER_H_ */
