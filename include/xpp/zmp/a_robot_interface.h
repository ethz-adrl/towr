/**
 @file    a_robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_ROBOT_INTERFACE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_ROBOT_INTERFACE_H_

#include <Eigen/Dense>

namespace xpp {
namespace zmp {

class ARobotInterface {
public:
  using PosXY = Eigen::Vector2d;

  ARobotInterface ();
  virtual ~ARobotInterface ();

  virtual PosXY GetNominalStanceInBase(int leg_id) const = 0;
  virtual double GetMaxDeviationXYFromNominal() const = 0;

};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_A_ROBOT_INTERFACE_H_ */
