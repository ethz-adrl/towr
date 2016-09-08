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

#include <xpp/zmp/com_motion.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/motion_structure.h>

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
  using Contacts = xpp::hyq::SupportPolygonContainer;
  using SuppPolygonPtrU = std::unique_ptr<xpp::hyq::SupportPolygonContainer>;

  using MotionPtrU      = std::unique_ptr<ComMotion>;

  InterpretingObserver (OptimizationVariables& subject);
  virtual ~InterpretingObserver ();

  void Init(const MotionStructure& structure,
            const ComMotion& com_motion,
            const Contacts& contacts);




  void SetInterpreter(const Interpreter&);

  void Update() override;
  VecSpline GetSplines() const;
  VecFoothold GetFootholds() const;
  VecFoothold GetStartStance() const;

  MotionPtrU GetComMotion() const      { return com_motion_->clone(); };
  Contacts GetContacts() const         { return *contacts_; };
  MotionStructure GetStructure() const { return motion_structure_; };





//  VectorXd GetMotionCoefficients() const { return  x_motion_; };
//  VectorXd GetContactCoefficients() const { return x_contacts_; };



private:
  MotionPtrU com_motion_;
  SuppPolygonPtrU contacts_;
  MotionStructure motion_structure_;

//  VectorXd x_motion_;
//  VectorXd x_contacts_;
  Interpreter interpreter_; // motion_ref remove
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_INTERPRETING_OBSERVER_H_ */
