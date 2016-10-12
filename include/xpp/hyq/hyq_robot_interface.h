/**
 @file    hyq_robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_ROBOT_INTERFACE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_ROBOT_INTERFACE_H_

#include "../opt/a_robot_interface.h"

namespace xpp {
namespace hyq {

class HyqRobotInterface : public xpp::opt::ARobotInterface{
public:
  HyqRobotInterface ();
  virtual ~HyqRobotInterface ();

  virtual PosXY GetNominalStanceInBase(int leg_id) const final;
  virtual MaxDevXY GetMaxDeviationXYFromNominal() const final;
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_ROBOT_INTERFACE_H_ */
