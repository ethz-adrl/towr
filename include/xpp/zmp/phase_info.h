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
namespace opt {

struct Contact {
  Contact() {};
  Contact(int _id, EndeffectorID _ee) : id(_id), ee(_ee) {}
  int id = -1; ///< a unique identifier for each contact, -1 if fixed by start
  EndeffectorID ee = EndeffectorID::E0;
};

inline std::ostream& operator<<(std::ostream& out, const Contact& c)
{
  out  << c.ee << ": id: " << c.id;
  return out;
}

/** Information to represent different types of motion.
  */
class PhaseInfo {
public:
  std::vector<Contact> free_contacts_; // all the stance legs currently in contact but not fixed by start
  std::vector<xpp::hyq::Foothold> fixed_contacts_;
  int id_ = 0;
  double duration_ = 0.0;
  int n_completed_steps_ = 0; // this is redundant, implicitly in the contacts ids

  PhaseInfo() {};

  /** @param type     Whether this is a stance, step of flight phase.
    * @param n_completed_steps how many steps completed by the previous phases.
    * @param id       Each phase has a unique ID.
    * @param duration How many seconds this phase lasts.
    */

  // for hyq 4 legs means stance
  bool IsStep() const { return (free_contacts_.size() + fixed_contacts_.size()) != 4;  }
};

inline std::ostream& operator<<(std::ostream& out, const PhaseInfo& p)
{
  out << "id: " << p.id_
      << "\t duration: " << p.duration_
      << "\n free contacts: ";

  for (auto c : p.free_contacts_)
    out << c << ";    ";

  out << "\n fixed contacts: ";
  for (auto c : p.fixed_contacts_)
    out << c << ";    ";

  return out;
}

using PhaseVec = std::vector<PhaseInfo>;

} /* namespace zmp */
} /* namespace xpp */


#endif /* USER_TASK_DEPENDS_XPP_OPT_INCLUDE_XPP_ZMP_PHASE_INFO_H_ */
