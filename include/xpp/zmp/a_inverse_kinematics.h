/**
 @file    a_inverse_kinematics.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Declares an interface to represent any inverse dynamics class.
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_INVERSE_KINEMATICS_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {

/** @brief An abstract class that every inverse dynamics class must conform with.
  *
  * This class is responsible for calculating the joint angles of a robot given
  * an endeffector position (=inverse kinematics).
  */
class AInverseKinematics {
public:
  typedef Eigen::VectorXd JointAngles;
  typedef Eigen::Vector3d EEPosition;

  AInverseKinematics ();
  virtual ~AInverseKinematics ();

  /** @brief Calculates the joint angles to reach for a specific endeffector position
    *
    * @param pos_b the 3d-position of the endeffector expressed in the base frame.
    * @param ee the number of endeffector that the above position is referring to.
    * @return the joints angles of the robot.
    */
  virtual JointAngles GetJointAngles(const EEPosition& pos_b, size_t ee) const = 0;

  /** @brief The upper and lower limit of the joints moving endeffector ee. */
  virtual JointAngles GetUpperJointLimits(size_t ee) const = 0;
  virtual JointAngles GetLowerJointLimits(size_t ee) const = 0;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_INVERSE_KINEMATICS_H_ */
