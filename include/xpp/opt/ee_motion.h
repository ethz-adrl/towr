/**
 @file    ee_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Declares the EEMotion class.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_

#include <xpp/opt/ee_swing_motion.h>

namespace xpp {
namespace opt {

/** Parametrizes the motion of one(!) endeffector swinging multiple times.
  */
class EEMotion {
public:
  struct Tlocal { double stance; double swing; };

  using Timings  = std::vector<Tlocal>;
  using Contacts = std::vector<Vector3d>;

public:
  EEMotion ();
  virtual ~EEMotion ();

  StateLin3d GetState(double t_global) const;
  bool IsInContact(double t_global) const;

  void SetParameters(const Timings&, const Contacts&);

private:

  // these are the parameters that fully describe the motion
  Timings timings_;
  Contacts contacts_;
  std::vector<EESwingMotion> swing_motions_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_ */
