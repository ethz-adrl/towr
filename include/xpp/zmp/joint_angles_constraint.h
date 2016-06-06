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

namespace xpp {
namespace zmp {

/** @brief Evaluates the implied joint angles for the current optimization values.
  *
  * This class is responsible for calculating the joint angles for the current
  * optimization variables and providing it's limits.
  */
class JointAnglesConstraint : public AConstraint, public IObserver {
public:
//  typedef OptimizationVariables::StdVecEigen2d FootholdsXY;

  JointAnglesConstraint ();
  virtual ~JointAnglesConstraint ();



private:
  VectorXd x_coeff_;
//  FootholdsXY footholds_;



//  OptimizationVariables* subject_;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_JOINT_ANGLES_CONSTRAINT_H_ */
