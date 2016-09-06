/**
 @file    hyq_robot_interface.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 6, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_ROBOT_INTERFACE_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_ROBOT_INTERFACE_H_

#include <xpp/zmp/a_robot_interface.h>

namespace xpp {
namespace zmp {

class HyqRobotInterface : public xpp::zmp::ARobotInterface{
public:
  HyqRobotInterface ();
  virtual ~HyqRobotInterface ();
};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_HYQ_HYQ_ROBOT_INTERFACE_H_ */
