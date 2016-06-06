/**
 @file    a_interpreting_observer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_DECORATOR_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_DECORATOR_H_

#include <xpp/zmp/observer_decorator.h>
#include <xpp/zmp/optimization_variables_interpreter.h>

namespace xpp {
namespace zmp {

class InterpretingDecorator : public ObserverDecorator {
public:
  typedef std::vector<xpp::zmp::ZmpSpline> VecSpline;
  typedef std::vector<xpp::hyq::Foothold>VecFoothold;
  typedef xpp::zmp::OptimizationVariablesInterpreter Interpreter;

  InterpretingDecorator (OptimizationVariables& subject, ObserverPtr& core);
  virtual ~InterpretingDecorator ();

  void Update();

private:
  Interpreter interpreter_;

  VecSpline splines_;
  VecFoothold footholds_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_DECORATOR_H_ */
