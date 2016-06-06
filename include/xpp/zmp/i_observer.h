/**
 @file    i_observer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Provides an abstract base class to implement the Observer in the
          observer pattern (https://sourcemaking.com/design_patterns/observer/cpp/3)
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_OBSERVER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_OBSERVER_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {

class OptimizationVariables; // forward declaration to avoid circular dependencies

class IObserver {
public:
  IObserver () {}; // fixme remove this, only for observer_visualizer
  IObserver (OptimizationVariables& subject);
  virtual ~IObserver ();

  virtual void Update() = 0;

protected:
  OptimizationVariables* subject_; ///< this variable holds the current state of optimization variables

private:
  // delete the copy and copy assignment operators, since that messes up the
  // logic with the subject in the observer pattern.
  IObserver& operator=(const IObserver&) = delete;
  IObserver(const IObserver&)            = delete;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_I_OBSERVER_H_ */
