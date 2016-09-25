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

/**
@brief Robot specific functions

All features specific for the hydraulic quadruped "HyQ" are located in this
namespace.
 */
namespace hyq {

/**
@brief Captures the full state of the robot (body, feet)
*/
class HyqState
{
public:
  typedef utils::Vec3d Vec3d;
  typedef utils::Point2d Pos2d;
  typedef utils::Point3d Point3d;
  typedef utils::Ori Ori;
  typedef utils::Coords3D Coords3D;
  typedef utils::Pose Pose;
  typedef std::vector<Foothold> VecFoothold;

public:
  HyqState();
  virtual ~HyqState();

  LegDataMap< bool > swingleg_;
  LegDataMap<Point3d> feet_;
  Pose base_; // geometric center of mass, vel, acc

  LegDataMap< Foothold > FeetToFootholds() const;
  Foothold FootToFoothold(LegID leg) const;

  const LegDataMap<Eigen::Vector3d> GetFeetPosOnly();
  VecFoothold GetStanceLegs() const;


//  /**
//   *@brief changes the swingleg to next following McGhee gait: LH, LF, RH, RF
//   */
//  void SwitchSwingleg();
  void SetSwingleg(LegID leg);

  std::array<Vec3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;

  void ZeroVelAcc();

  int SwinglegID() const;
private:
};

class HyqStateStamped : public HyqState {
public:
  double t;
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
