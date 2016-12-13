/**
@file    motion_phase.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 3, 2016
@brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_PHASE_INFO_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_OPT_PHASE_INFO_H_

#include <xpp/utils/eigen_std_conversions.h>
#include <xpp/opt/contact.h>
#include <vector>

namespace xpp {
namespace opt {

/** Represents a distinct phase in the motion with fixed system dynamics.
  *
  * This means the contact configuration (endeffectors is stance) is fixed.
  */
class MotionPhase {
public:
  using FootholdVec = std::vector<Contact>;
  using ContactVec  = std::vector<ContactBase>;

  ContactVec free_contacts_;       ///< all the ee currently in contact but not fixed by start stance
  ContactVec swing_goal_contacts_; ///< the contacts the current swinglegs are swinging towards
  FootholdVec fixed_contacts_;     ///< the contacts fixed by start stance
  double duration_ = 0.0;          ///< how long [s] this phase lasts

  bool IsStep() const { return !swing_goal_contacts_.empty();  }


  ContactVec GetAllContacts() const;
  FootholdVec GetAllContacts(const utils::StdVecEigen2d& contacts_xy) const;
};

class MotionPhaseStamped : public MotionPhase {
public:
  MotionPhaseStamped(const MotionPhase& base) : MotionPhase(base) {};
  MotionPhaseStamped() : time_(0.0) {};
  double time_ = 0.0;
};


inline std::ostream& operator<<(std::ostream& out, const MotionPhase& p)
{
  out << "\t duration: " << p.duration_
      << "\n free contacts: ";

  for (auto c : p.free_contacts_)
    out << c << ";    ";

  out << "\n fixed contacts: ";
  for (auto c : p.fixed_contacts_)
    out << c << ";    ";

  out << "\n swing goal contacts: ";
  for (auto c : p.swing_goal_contacts_)
    out << c << ";    ";

  return out;
}


} /* namespace opt */
} /* namespace xpp */


#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_PHASE_INFO_H_ */
