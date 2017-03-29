/**
 @file    ee_motion.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Declares the EEMotion class.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_

#include <xpp/opt/ee_swing_motion.h>
#include <xpp/contact.h>
#include <deque>

namespace xpp {
namespace opt {

/** Parametrizes the motion of one(!) endeffector swinging multiple times.
  */
class EEMotion {
public:
  using ContactPositions = std::deque<Contact>;

  EEMotion ();
  virtual ~EEMotion ();


  void SetInitialPos(const Vector3d& pos, EndeffectorID);
  void AddStancePhase(double t);
  void AddSwingPhase(double t, const Vector3d& goal);
  void UpdateContactPosition(int foothold_of_leg, const Vector3d& pos);


  StateLin3d GetState(double t_global) const;
  bool IsInContact(double t_global) const;
  /** Empty vector is no contact at that time */
  ContactPositions GetContact(double t_global) const;
  double GetTotalTime() const;
  /** Those not fixed by the start stance
    */
  ContactPositions GetContacts() const;
  EndeffectorID GetEE() const;


private:
  int GetPhase(double t_global) const;
  void AddPhase(double t, const Vector3d& goal, double lift_height = 0.03);

  void UpdateSwingMotions();

  EndeffectorID ee_;
  ContactPositions contacts_;
  std::deque<bool> is_contact_phase_;
  std::vector<EESwingMotion> phase_motion_;

};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_EE_MOTION_H_ */
