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
  for (const auto& c_free : free_contacts_)
    contacts.push_back(c_free);

  for (const auto& c_fixed : fixed_contacts_)
    contacts.push_back(c_fixed); // strips away child info

  return contacts;
}

// think about caching this in CalcPhaseStampedVec for all nodes
MotionPhase::FootholdVec
MotionPhase::GetAllContacts(const utils::StdVecEigen2d& contacts_xy) const
{
  FootholdVec contacts;

  for (const auto& c_free : free_contacts_) {
    Contact contact(c_free);
    contact.p.x() = contacts_xy.at(c_free.id).x();
    contact.p.y() = contacts_xy.at(c_free.id).y();
    contact.p.z() = 0.0;
    contacts.push_back(contact);
  }

  for (const auto& c_fixed : fixed_contacts_) {
    contacts.push_back(c_fixed);
  }

  return contacts;
}



} // namespace opt
} // namespace xpp
