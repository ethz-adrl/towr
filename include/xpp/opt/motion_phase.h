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
  using EEID        = utils::EndeffectorID;

  ContactVec contacts_opt_;       ///< all the ee currently in contact but not fixed by start stance
  ContactVec swinglegs_; ///< the contacts the current swinglegs are swinging towards
  FootholdVec contacts_fixed_;     ///< the contacts fixed by start stance
  double duration_ = 0.0;          ///< how long [s] this phase lasts

  bool IsStep() const { return !swinglegs_.empty();  }

  void RemoveContact(EEID ee);
  void ShiftSwingToStance(EEID ee);
  bool IsInSwinglegs(EEID ee);


  /** @returns fixed and free current contacts without xyz-positions.
    */
  ContactVec GetAllContacts() const;

  /** @returns fixed and free current contacts including xyz-positions.
    */
  FootholdVec GetAllContacts(const utils::StdVecEigen2d& contacts_xy) const;
};

class MotionNode : public MotionPhase {
public:
  MotionNode(const MotionPhase& base) : MotionPhase(base) {};
  MotionNode() : time_(0.0) {};
  double time_ = 0.0;
};


inline std::ostream& operator<<(std::ostream& out, const MotionPhase& p)
{
  out << " duration: " << p.duration_
      << "\n free contacts: ";

  for (auto c : p.contacts_opt_)
    out << c << ";    ";

  out << "\n fixed contacts: ";
  for (auto c : p.contacts_fixed_)
    out << c << ";    ";

  out << "\n swing goal contacts: ";
  for (auto c : p.swinglegs_)
    out << c << ";    ";

  return out;
}


} /* namespace opt */
} /* namespace xpp */


#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_PHASE_INFO_H_ */
