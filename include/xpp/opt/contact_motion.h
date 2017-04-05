/**
 @file    contact_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Apr 5, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CONTACT_MOTION_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CONTACT_MOTION_H_

#include <xpp/parametrization.h>

namespace xpp {
namespace opt {

/** @brief Knows which endeffector is in contact at time t during trajectory.
 */
class ContactMotion : public Parametrization {
public:
  ContactMotion ();
  virtual ~ContactMotion ();
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CONTACT_MOTION_H_ */
