/**
@file    motion_phase.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Dec 13, 2016
@brief   Brief description
 */

#include <xpp/opt/motion_phase.h>

namespace xpp {
namespace opt {


// these should be ordered same as below (free before fixed)
MotionPhase::ContactVec
MotionPhase::GetAllContacts() const
{
  ContactVec contacts;
  for (const auto& c_free : contacts_opt_)
    contacts.push_back(c_free);

  for (const auto& c_fixed : contacts_fixed_)
    contacts.push_back(c_fixed); // strips away child info

  return contacts;
}

// think about caching this in CalcPhaseStampedVec for all nodes
MotionPhase::FootholdVec
MotionPhase::GetAllContacts(const utils::StdVecEigen2d& contacts_xy) const
{
  FootholdVec contacts;

  for (const auto& c_free : contacts_opt_) {
    Contact contact(c_free);
    contact.p.x() = contacts_xy.at(c_free.id).x();
    contact.p.y() = contacts_xy.at(c_free.id).y();
    contact.p.z() = 0.0;
    contacts.push_back(contact);
  }

  for (const auto& c_fixed : contacts_fixed_) {
    contacts.push_back(c_fixed);
  }

  return contacts;
}

void
MotionPhase::RemoveContact (EEID ee)
{
  // remove ee from footholds fixed at start
  auto it_fixed = std::find_if(contacts_fixed_.begin(), contacts_fixed_.end(),
                               [&](const Contact& f) {return f.ee == ee;});

  if (it_fixed != contacts_fixed_.end()) // step found in initial stance
    contacts_fixed_.erase(it_fixed);     // remove contact, because now swinging leg


  // remove ee from last free contacts
  auto it_free = std::find_if(contacts_opt_.begin(), contacts_opt_.end(),
                              [&](const ContactBase& c) {return c.ee == ee;});

  if (it_free != contacts_opt_.end()) // step found in current stance
    contacts_opt_.erase(it_free);     // remove contact, because now swinging leg
}

void
MotionPhase::ShiftSwingToStance (EEID ee)
{
  // remove ee from last free contacts
  auto it = std::find_if(swinglegs_.begin(), swinglegs_.end(), [&](const ContactBase& c) {return c.ee == ee;});
  contacts_opt_.push_back(*it);
  swinglegs_.erase(it);
}

bool
xpp::opt::MotionPhase::IsInSwinglegs (EEID ee)
{
  auto it = std::find_if(swinglegs_.begin(), swinglegs_.end(), [&](const ContactBase& c) {return c.ee == ee;});
  return (it != swinglegs_.end());
}

} // namespace opt
} // namespace xpp

