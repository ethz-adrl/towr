/**
 @file    observer_visualizer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 1, 2016
 @brief   Declares an interface for a class that observes optimization variables
          and visualizes them.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_OBSERVER_VISUALIZER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_OBSERVER_VISUALIZER_H_

#include <xpp/zmp/i_observer.h>
#include <xpp/ros/i_visualizer.h>
#include <xpp/zmp/optimization_variables.h>

namespace xpp {
namespace zmp {

// fixme make this class visualizer, but delegate/compose observer responsibilities,
// not inheritance
class AObserverVisualizer : public IObserver, public ros::IVisualizer {
public:
  AObserverVisualizer ();
  virtual ~AObserverVisualizer ();

  virtual void RegisterWithSubject(OptimizationVariables& subject) = 0;
};

class DoNothingObserverVisualizer : public AObserverVisualizer {
  void Update() { /* does nothing */ };
  void RegisterWithSubject(OptimizationVariables& subject) { /* does nothing */ };
};

// compilation unit scope object that can be used as default initialization
// "static" : each .cc files that includes this header has it's own copy of this variable
static DoNothingObserverVisualizer do_nothing_observer_visualizer;

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_OBSERVER_VISUALIZER_H_ */
