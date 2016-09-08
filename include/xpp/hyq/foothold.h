/**
@file    foothold.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   A foot position (x,y,z) and an associated leg.
 */

#ifndef _XPP_HYQ_FOOTHOLD_H_
#define _XPP_HYQ_FOOTHOLD_H_

#include <xpp/hyq/leg_data_map.h> // LegID, LegIDArray
#include <xpp/utils/geometric_structs.h>

#include <map>
#include <iostream>

namespace xpp {
namespace hyq {

/**
@brief A foot position (x,y,z) and an associated leg.
*/
class Foothold {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::vector<Foothold> VecFoothold;
  typedef xpp::utils::StdVecEigen2d StdVecEigen2d;
  typedef Eigen::Vector2d Vector2d;

public:
  Eigen::Vector3d p;
  LegID leg;
  int id; // the number of this foothold in a sequence of footholds, -1 for start stance

  // refactor remove this from here and move to contact
  static const int kFixedByStart = -1;

  Foothold();
  Foothold(Eigen::Vector3d _pos, LegID _leg);
  Foothold(double x, double y, double z, LegID _leg);

  Vector2d GetXy() const;
  void SetXy(const Vector2d& xy);
  static void SetXy(const StdVecEigen2d& xy, VecFoothold& footholds);

  /**  @brief returns true if a foothold with that leg is present in footholds */
  static bool IsInFootholds(LegID leg, const VecFoothold& footholds);
  /** @brief  searches through footholds looking for leg starting from most current ones */
  static Foothold GetLastFoothold(LegID leg, const VecFoothold& footholds);
  /** @brief finds the last foothold with that leg and updates the x,y,z-position */
  static void UpdateFoothold(const Foothold& f_new, VecFoothold& footholds);
  /** @brief returns the position of the last foothold with that leg */
  static int GetLastIndex(LegID leg, const VecFoothold& footholds);

  bool operator==(const Foothold& rhs) const;
  bool operator!=(const Foothold& rhs) const;
};


inline std::ostream& operator<<(std::ostream& out, const Foothold& f)
{
  std::map<LegID, std::string> l {
    { LF, "LF" }, { RF, "RF" }, { LH, "LH" }, { RH, "RH" }
  };

  out << l[f.leg] << " : " << f.p.transpose() << ", id: " << f.id;
  return out;
}


} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_FOOTHOLD_H_
