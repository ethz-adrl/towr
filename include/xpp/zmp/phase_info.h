/**
@file    phase_info.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Sep 3, 2016
@brief   Brief description
 */

#ifndef USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_
#define USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_

#include "a_robot_interface.h"
#include <xpp/hyq/foothold.h>

#include <iostream>
#include <vector>

namespace xpp {
namespace zmp {

struct Contact {
  Contact(int _id, EndeffectorID _ee) : id(_id), ee(_ee) {}
  int id; ///< a unique identifier for each contact, -1 if fixed by start
  EndeffectorID ee;
};

inline std::ostream& operator<<(std::ostream& out, const Contact& c)
{
  out << "id: " << c.id << ", ee: " << c.ee;
  return out;
}

/** Information to represent different types of motion.
  */
// motion_ref augment phase info with id's of legs in contact
class PhaseInfo {
public:
  enum Type {kStancePhase=0, kStepPhase, kFlightPhase} type_; // replace this with if contacts=4=total number of legs
  int n_completed_steps_; // this is also not needed anymore, implicitly in the contacts

  std::vector<Contact> contacts_; // all the stance legs currently in contact but not fixed by start
  std::vector<xpp::hyq::Foothold> fixed_contacts_;
  int id_;
  double duration_;

  PhaseInfo() : type_(kStancePhase), id_(-1), duration_(0.0) {};

  /** @param type     Whether this is a stance, step of flight phase.
    * @param n_completed_steps how many steps completed by the previous phases.
    * @param id       Each phase has a unique ID.
    * @param duration How many seconds this phase lasts.
    */
  PhaseInfo(Type type, int id, double duration)
    : type_(type), id_(id),
      duration_(duration) {};
};

inline std::ostream& operator<<(std::ostream& out, const PhaseInfo& p)
{
  out << "id: " << p.id_
      << "\t type: " << p.type_
      << "\t duration: " << p.duration_
      << "\n free contacts: ";

  for (auto c : p.contacts_)
    out << c << "\t";

  out << "\n fixed contacts: ";
  for (auto c : p.fixed_contacts_)
    out << c << "\t";

  return out;
}

using PhaseVec = std::vector<PhaseInfo>;

} /* namespace zmp */
} /* namespace xpp */


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_ */
