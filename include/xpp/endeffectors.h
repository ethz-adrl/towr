/**
 @file    endeffectors.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 2, 2017
 @brief   Declares the generic Endeffector class
 */

#ifndef XPP_INCLUDE_XPP_ENDEFFECTORS_H_
#define XPP_INCLUDE_XPP_ENDEFFECTORS_H_

#include <deque>
#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>

namespace xpp {

enum EndeffectorID { E0, E1, E2, E3, E4, E5 };

/** @brief Data structure to assign values to each endeffector.
 *
 *  Common values are xyz-positions (Vector3d), contact-flag (bool).
 */
template<typename T>
class Endeffectors {
public:
  using Container     = std::deque<T>; // to avoid faulty std::vector<bool>
  using EndeffectorsT = Endeffectors<T>;

  Endeffectors (int n_ee = 0);
  virtual ~Endeffectors ();

  /** @brief Defines the number of endeffectors.
   */
  void SetCount(int n_ee);

  /** @brief Sets each endeffector to value.
   */
  void SetAll(const T& value);

  /** @returns The number of endeffectors this structure holds.
   */
  int GetCount() const;

  /** @returns all endeffector IDs from E0->EN.
   */
  std::vector<EndeffectorID> GetEEsOrdered() const;

  T& At(EndeffectorID ee);
  const T& At(EndeffectorID ee) const;

  const EndeffectorsT operator-(const EndeffectorsT& rhs) const;
  const EndeffectorsT operator/(double scalar) const;
  bool operator!=(const Endeffectors& other) const;

  /** @returns a returns a read-only copy of the underlying STL-container.
   */
  const Container ToImpl() const;


private:
  Container ee_;
};

using EndeffectorsPos  = Endeffectors<Eigen::Vector3d>;
using EndeffectorsVel  = EndeffectorsPos;


template <typename T>
std::ostream& operator<<(std::ostream& stream, Endeffectors<T> endeffectors)
{
  for (EndeffectorID ee : endeffectors.GetEEsOrdered())
    stream << endeffectors.At(ee) << ", ";

  return stream;
}


class EndeffectorsBool : public Endeffectors<bool> {
public:
  EndeffectorsBool (int n_ee = 0, bool val = false) :Endeffectors(n_ee) { SetAll(val);};
  virtual ~EndeffectorsBool () {};

  /** @brief returns a copy with flipped boolean values. */
  EndeffectorsBool Invert() const
  {
    EndeffectorsBool ret(GetCount());
    for (auto ee : GetEEsOrdered())
      ret.At(ee) = !At(ee);

    return ret;
  }

  /** @brief number of endeffectors with flag set to TRUE */
  int GetTrueCount() const
  {
    int count = 0;
    for (auto ee : GetEEsOrdered())
      if (At(ee))
        count++;

    return count;
  }
};



namespace biped {
static const std::string L = "L";
static const std::string R = "R";
static const std::map<std::string, EndeffectorID> kMapIDToEE {
  { L, E0},
  { R, E1},
};
}
namespace quad {
static const std::string LF = "LF";
static const std::string RF = "RF";
static const std::string LH = "LH";
static const std::string RH = "RH";
static const std::map<std::string, EndeffectorID> kMapIDToEE {
  { LF, E0},
  { RF, E1},
  { LH, E2},
  { RH, E3},
};
}
namespace quad_rotor { // underscore to not collide with biped definitions
static const std::string L = "L";
static const std::string R = "R";
static const std::string F = "F";
static const std::string H = "H";
static const std::map<std::string, EndeffectorID> kMapIDToEE {
  { L, E0},
  { R, E1},
  { F, E2},
  { H, E3},
};
}


template<typename T>
static std::map<EndeffectorID, T> ReverseMap(std::map<T, EndeffectorID> map) {

  std::map<EndeffectorID, T> reverse;
  for (auto p : map) {
    auto flipped = std::pair<EndeffectorID, T>(p.second, p.first);
    if ( !reverse.insert(flipped).second )
      assert(false); //  key already present, won't overwrite...
  }

  return reverse;
}



//// the mapping of these strings to endeffector values
//static const std::map<std::string, EndeffectorID> kMapIDToEE {
//  { biped::L, E0},
//  { biped::R, E1},
//
//  { quad::LF, E0},
//  { quad::RF, E1},
//  { quad::LH, E2},
//  { quad::RH, E3},
//
//  { quad_rotor::L, E0},
//  { quad_rotor::R, E1},
//  { quad_rotor::F, E2},
//  { quad_rotor::H, E3},
//};


//// some specific morphologies
//namespace biped {
////enum FootID { L, R };
////
////static const std::map<EndeffectorID, FootID> kMapOptToBiped {
////  { EndeffectorID::E0,  L},
////  { EndeffectorID::E1,  R},
////};
//} // namespace biped
//
//namespace quad {
////enum FootID { RF, LF, LH, RH };
//
////static const std::map<EndeffectorID, FootID> kMapOptToQuad {
////  { EndeffectorID::E0,  LF},
////  { EndeffectorID::E1,  RF},
////  { EndeffectorID::E2,  LH},
////  { EndeffectorID::E3,  RH},
////};
//
////enum RotorID { L, R, F, H };
////static const std::map<EndeffectorID, RotorID> kMapOptToRotor {
////  { EndeffectorID::E0,  L},
////  { EndeffectorID::E1,  F},
////  { EndeffectorID::E2,  R},
////  { EndeffectorID::E3,  H},
////};
//
//} // namespace quadruped


//// remove this one
//template<typename T>
//static std::map<T, EndeffectorID> Reverse(std::map<EndeffectorID, T> map) {
//
//  std::map<T, EndeffectorID> reverse;
//  for (auto pair : map) {
//    auto ee = pair.first;
//    reverse[map.at(ee)] = ee;
//  }
//
//  return reverse;
//}



} /* namespace xpp */

#include "impl/endeffectors-impl.h"

#endif /* XPP_INCLUDE_XPP_ENDEFFECTORS_H_ */
