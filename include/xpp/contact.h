/**
@file    contact.h
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Dec 13, 2016
@brief   Brief description
 */

#ifndef XPP_INCLUDE_XPP_CONTACT_H_
#define XPP_INCLUDE_XPP_CONTACT_H_

#include <xpp/endeffectors.h>
#include <Eigen/Dense>
#include <iostream>

namespace xpp {

class ContactBase {
public:
  ContactBase() {};

  /** @param _id Zero if this contact is established by the start stance, 1 if
    *            after first step of this endeffector,...
    * @param _ee The endeffector used to establish this contact.
    */
  ContactBase(int _id, EndeffectorID _ee) : id(_id), ee(_ee) {}

  int id = 0; ///< a unique identifier for each contact,
  EndeffectorID ee = EndeffectorID::E0;
};

class Contact : public ContactBase {
public:
  using Vector3d = Eigen::Vector3d;

  Contact(){};
  Contact(int id, EndeffectorID ee, const Vector3d& pos)
     :ContactBase(id,ee) {
    p = pos;
  }

  Contact(const ContactBase& base) : ContactBase(base) {};
  Vector3d p;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator<<(std::ostream& out, const Contact& c)
{
  out  << "{ee:" << c.ee << ",id:" << c.id << ",p:" << c.p.transpose()  << "}";
  return out;
}

} /* namespace xpp */

#endif /* XPP_XPP_COMMON_INCLUDE_XPP_UTILS_CONTACT_H_ */
