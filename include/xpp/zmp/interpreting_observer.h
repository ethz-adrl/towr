/**
 @file    interpreting_observer.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 8, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_

#include <xpp/zmp/i_observer.h>
#include <xpp/zmp/optimization_variables.h>
#include <xpp/zmp/optimization_variables_interpreter.h>


namespace xpp {
namespace hyq {
class Foothold;
}
}

namespace xpp {
namespace zmp {

class OptimizationVariablesInterpreter;
class ComPolynomial;

/** @brief Puts the optimization variables into context
  *
  * This class is responsible for observing the current values of the
  * optimization variables and interpreting them based on the problem structure.
  */
class InterpretingObserver : public IObserver {
public:
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef std::vector<ComPolynomial> VecSpline;
  typedef OptimizationVariablesInterpreter Interpreter;

  InterpretingObserver (OptimizationVariables& subject);
  virtual ~InterpretingObserver ();

  void SetInterpreter(const Interpreter&);

  void Update() override;
  VecSpline GetSplines() const;
  VecFoothold GetFootholds() const;
  VecFoothold GetStartStance() const;

private:
  Interpreter interpreter_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_ */
