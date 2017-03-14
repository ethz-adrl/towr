/**
 @file    endeffectors_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_

#include <xpp/opt/ee_motion.h>
#include <xpp/state.h>
#include <xpp/endeffectors.h>

namespace xpp {
namespace opt {

/** Represents the motion of all the endeffectors (feet, hands) of a system
  */
class EndeffectorsMotion {
public:
  using EEState = Endeffectors<StateLin3d>;

  EndeffectorsMotion ();
  virtual ~EndeffectorsMotion ();


  void SetInitialPos(const EEXppPos& initial_pos);


  EEMotion& GetMotion(EndeffectorID ee);


  EEState GetEndeffectors(double t_global) const;

  // something about contact state, see how most used

private:
  Endeffectors<EEMotion> endeffectors_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTORS_MOTION_H_ */
