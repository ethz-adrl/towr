/**
@file    hyq_state.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Captures the full state of the robot (body, feet)
 */

#ifndef _XPP_HYQ_STATE_H_
#define _XPP_HYQ_STATE_H_

#include "foothold.h"
#include "leg_data_map.h"

#include <xpp/utils/geometric_structs.h>

namespace xpp {
namespace hyq {

/** Captures the full state of the robot (body, feet)
  */
class HyqState {
public:
  typedef utils::Pose Pose;
  typedef utils::Point3d Point3d;
  typedef Eigen::Vector3d Vector3d;
  typedef std::vector<Foothold> VecFoothold;

  HyqState();
  virtual ~HyqState();

  LegDataMap< bool > swingleg_;
  LegDataMap<Point3d> feet_;
  Pose base_; // geometric center of mass, vel, acc

  // cmo this might not be necessary, return position and stance independent?
  LegDataMap< Foothold > FeetToFootholds() const;
  Foothold FootToFoothold(LegID leg) const;
  VecFoothold GetStanceLegs() const;

  const LegDataMap<Vector3d> GetFeetPosOnly();

  void SetSwingleg(LegID leg);
  std::array<Vector3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;
  void ZeroVelAcc();
  int SwinglegID() const;
};

class HyqStateStamped : public HyqState {
public:
  double t_;
};


inline std::ostream& operator<<(std::ostream& out, const HyqState& hyq)
{
  out << "base: " << hyq.base_ << "\n"
      << "feet: " << "\tLF = " <<  hyq.feet_[LF] << "\n"
                  << "\tRF = " <<  hyq.feet_[RF] << "\n"
                  << "\tLH = " <<  hyq.feet_[LH] << "\n"
                  << "\tRH = " <<  hyq.feet_[RH] << "\n"
      << "swing:\t" << hyq.swingleg_ << "\n";
   return out;
}


} // namespace hyq
} // namespace xpp

#endif // _XPP_HYQ_STATE_H_
