/**
 @file    obstacle_constraint.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 20, 2016
 @brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_

#include <xpp/zmp/a_constraint.h>

namespace xpp {
namespace zmp {

class ObstacleConstraint : public AConstraint {
public:
  ObstacleConstraint ();
  virtual ~ObstacleConstraint ();


};

} /* namespace zmp */
} /* namespace xpp */

#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_OBSTACLE_CONSTRAINT_H_ */
