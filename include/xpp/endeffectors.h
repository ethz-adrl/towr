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
//  EndeffectorsBool Invert() const
//  {
//    EndeffectorsBool ret(GetCount());
//    for (auto ee : GetEEsOrdered())
//      ret.At(ee) = !At(ee);
//
//    return ret;
//  }

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




// the implementation of the above functions
template<typename T>
Endeffectors<T>::Endeffectors (int n_ee)
{
  SetCount(n_ee);
}

template<typename T>
void
Endeffectors<T>::SetCount (int n_ee)
{
  ee_.resize(n_ee);
}

template<typename T>
Endeffectors<T>::~Endeffectors ()
{
}

template<typename T>
void
Endeffectors<T>::SetAll (const T& value)
{
  std::fill(ee_.begin(), ee_.end(), value);
}

template<typename T>
T&
Endeffectors<T>::At (EndeffectorID idx)
{
  return ee_.at(idx);
}

template<typename T>
const T&
Endeffectors<T>::At (EndeffectorID idx) const
{
  return ee_.at(idx);
}

template<typename T>
int
Endeffectors<T>::GetCount () const
{
  return ee_.size();
}

template<typename T>
const typename Endeffectors<T>::Container
Endeffectors<T>::ToImpl () const
{
  return ee_;
}

template<typename T>
const typename Endeffectors<T>::EndeffectorsT
Endeffectors<T>::operator - (const EndeffectorsT& rhs) const
{
  EndeffectorsT result(ee_.size());
  for (auto i : GetEEsOrdered())
    result.At(i) = ee_.at(i) - rhs.At(i);

  return result;
}

template<typename T>
const typename Endeffectors<T>::EndeffectorsT
Endeffectors<T>::operator / (double scalar) const
{
  EndeffectorsT result(ee_.size());
  for (auto i : GetEEsOrdered())
    result.At(i) = ee_.at(i)/scalar;

  return result;
}

template<typename T>
std::vector<EndeffectorID>
Endeffectors<T>::GetEEsOrdered () const
{
  std::vector<EndeffectorID> vec;
  for (int i=0; i<ee_.size(); ++i)
    vec.push_back(static_cast<EndeffectorID>(i));

  return vec;
}

template<typename T>
bool
Endeffectors<T>::operator!=(const Endeffectors& other) const
{
  for (auto ee : GetEEsOrdered()) {
    if (ee_.at(ee) != other.At(ee))
      return true;
  }
  return false;
}

} /* namespace xpp */

#endif /* XPP_INCLUDE_XPP_ENDEFFECTORS_H_ */
