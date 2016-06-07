/**
 @file    joint_angles_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_JOINT_ANGLES_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_JOINT_ANGLES_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>
#include <xpp/zmp/i_observer.h>

#include <xpp/zmp/optimization_variables_interpreter.h>
#include <xpp/zmp/stance_feet_calculator.h>
#include <xpp/zmp/a_inverse_kinematics.h>

namespace xpp {
namespace zmp {

/** @brief Evaluates the implied joint angles for the current optimization values.
  *
  * This class is responsible for calculating the joint angles for the current
  * optimization variables and providing it's limits.
  */
class JointAnglesConstraint : public AConstraint, public IObserver {
public:
  typedef xpp::utils::StdVecEigen2d FootholdsXY;
  typedef xpp::zmp::OptimizationVariablesInterpreter Interpreter;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef std::vector<ZmpSpline> VecSpline;
  typedef AInverseKinematics::JointAngles JointAngles3d;

  JointAnglesConstraint (OptimizationVariables& subject);
  virtual ~JointAnglesConstraint ();

  void Init(const Interpreter& interpreter, const VecFoothold& start_stance);
  void Update() override;

  VectorXd EvaluateConstraint() const override;
  VecBound GetBounds() const override;

private:
  AInverseKinematics* inv_kin_;           ///< endeffector to joint angle conversions
  StanceFeetCalculator stance_feet_calc_; ///< supplies feet position in base frame

  VecFoothold start_stance_;
  Interpreter interpreter_; ///< adds context to the optimization variables

  std::vector<double> vec_t_; ///< evaluates constraint at each of these times
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_JOINT_ANGLES_CONSTRAINT_H_ */
