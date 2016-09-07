/**
 @file    a_robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 6, 2016
 @brief   Declares the class RobotInterface
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_ROBOT_INTERFACE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_ROBOT_INTERFACE_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {

/** @brief Abstracts all robot specific values
  *
  * This is the interface that the optimization code is programmed against.
  * To use a specific robot, derive from this class and pass that object to
  * the required costs/constraints.
  */
class ARobotInterface {
public:
  using PosXY = Eigen::Vector2d;

  ARobotInterface ();
  virtual ~ARobotInterface ();

  /** @brief default contact position of the endeffectors
    */
  virtual PosXY GetNominalStanceInBase(int leg_id) const = 0;

  /** @How much the Endeffector can deviate from the default (x,y) position
    * while still remaining in the range of motion.
    *
    * Used by RangeOfMotionConstraint.
    */
  virtual double GetMaxDeviationXYFromNominal() const = 0;

};


enum class EndeffectorID { E0, E1, E2, E3, E4, E5 };

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_ROBOT_INTERFACE_H_ */
