/**
 @file    articulated_robot_state_cartesian.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_COMMON_INCLUDE_XPP_OPT_ROBOT_STATE_CARTESIAN_H_
#define XPP_XPP_COMMON_INCLUDE_XPP_OPT_ROBOT_STATE_CARTESIAN_H_

#include <xpp/state.h>
#include <xpp/endeffectors.h>

namespace xpp {

class RobotStateCartesian {
public:
  RobotStateCartesian(int n_ee);
  ~RobotStateCartesian() {};

  State3d GetBase() const { return base_; };
  Endeffectors<Vector3d> GetEEPos() const;
  Endeffectors<Vector3d> GetEEAcc() const;
  EndeffectorsBool GetContactState() const { return ee_contact_; };

  State3d base_;
  double t_global_;
  Endeffectors<StateLin3d> ee_motion_;
  EndeffectorsBool ee_contact_;
  Endeffectors<Vector3d> ee_forces_;
};

} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_OPT_ROBOT_STATE_CARTESIAN_H_ */
