/**
@file    hyq_state.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Captures the full state of the robot (body, feet)
 */

#ifndef HYQ_STATE_H_
#define HYQ_STATE_H_

#include "foothold.h"
#include "leg_data_map.h"

#include <xpp/utils/geometric_structs.h>

#include <log4cxx/logger.h>

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
  typedef utils::Point3d Pos;
  typedef utils::Ori Ori;
  typedef utils::Coords3D Coords3D;
  typedef utils::Pose Pose;

public:
  bool swing_phase_;
  LegDataMap< bool > swingleg_;
  LegDataMap<Pos> feet_;
  Pose base_; // geometric center of mass, vel, acc


public:
  HyqState();
  virtual ~HyqState();


  LegDataMap< Foothold > FeetToFootholds() const;
  Foothold FootToFoothold(LegID leg) const;

  const LegDataMap<Eigen::Vector3d>& GetFeetPosOnly();

  /**
   *@brief changes the swingleg to next following McGhee gait: LH, LF, RH, RF
   */
  void SwitchSwingleg();
  void SetSwingleg(LegID leg);
  int SwinglegID() const;

  std::array<Vec3d, kNumSides> GetAvgSides() const;
  double GetZAvg() const;

  /**
   * @brief smoothly changes the 2d-position of the body to desired one
   *
   * @param des desired 2d-position to set base to
   * @param t_local time since splining began
   * @param t_max if t_local reaches this time, the desired pos must be reached
   */
  void RampInPos(double des, Coords3D coord, double t_local, double t_max);
  void ZeroVelAcc();

private:
  static log4cxx::LoggerPtr log_;
};


#include "hyq_state-inl.h"


} // namespace hyq
} // namespace xpp

#endif /* HYQ_STATE_H_ */
