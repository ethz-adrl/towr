/**
 @file    a_robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 6, 2016
 @brief   Declares the class RobotInterface
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_A_ROBOT_INTERFACE_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_A_ROBOT_INTERFACE_H_

#include <xpp/utils/endeffectors.h>
#include <Eigen/Dense>
#include <array>

namespace xpp {
namespace opt {

/** @brief Abstracts all robot specific values
  *
  * This is the interface that the optimization code is programmed against.
  * To use a specific robot, derive from this class and pass that object to
  * the required costs/constraints.
  */
// zmp_ remove this
class ARobotInterface {
public:
  using PosXY    = Eigen::Vector2d;
  using MaxDevXY = std::array<double,2>;
  using EndeffectorID = xpp::utils::EndeffectorID;

  ARobotInterface () {};
  virtual ~ARobotInterface () {};

  /** @brief default contact position of the endeffectors
    */
  virtual PosXY GetNominalStanceInBase(EndeffectorID leg_id) const = 0;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_A_ROBOT_INTERFACE_H_ */
