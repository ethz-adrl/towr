/**
 @file    joint_angles_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_JOINT_ANGLES_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_JOINT_ANGLES_CONSTRAINT_H_

#include <xpp/zmp/optimization_variables_interpreter.h>
#include <memory>

#include "../opt/a_constraint.h"
#include "../opt/a_inverse_kinematics.h"
#include "../opt/motion_structure.h"

namespace xpp {
namespace opt {

/** @brief Evaluates the implied joint angles for the current optimization values.
  *
  * This class is responsible for calculating the joint angles for the current
  * optimization variables and providing it's limits.
  */
class JointAnglesConstraint : public AConstraint {
public:
  typedef xpp::utils::StdVecEigen2d FootholdsXY;
  typedef xpp::opt::OptimizationVariablesInterpreter Interpreter;
  typedef std::vector<xpp::hyq::Foothold> VecFoothold;
  typedef std::vector<ComPolynomial> VecSpline;
  typedef AInverseKinematics::JointAngles JointAngles;
  typedef xpp::hyq::Foothold Foothold;
  typedef std::shared_ptr<AInverseKinematics> InvKinPtr;

  JointAnglesConstraint ();
  virtual ~JointAnglesConstraint ();

  void Init(const Interpreter& interpreter, const InvKinPtr& inv_kin);
  void UpdateVariables(const OptimizationVariables*) override;

  VectorXd EvaluateConstraint() const override;
  VecBound GetBounds() const override;
  // not implemented yet
  Jacobian GetJacobianWithRespectTo (std::string var_set) const override { assert(false); };

private:
  InvKinPtr inv_kin_;           ///< endeffector to joint angle conversions
  MotionStructure stance_feet_calc_; ///< supplies feet position in base frame

  Interpreter interpreter_; ///< adds context to the optimization variables

  std::vector<double> vec_t_; ///< evaluates constraint at each of these times
};

} /* namespace zmp */
} /* namespace xpp */



#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_JOINT_ANGLES_CONSTRAINT_H_ */
