/**
@file    hyq_state.cc
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Captures the full state of the robot (body, feet)
 */

#include <xpp/hyq/hyq_state.h>

namespace xpp {
namespace hyq {

using ::xpp::utils::Z;
typedef utils::Point2d Pos2d;
typedef utils::Ori Ori;
typedef utils::Coords3D Coords3D;

HyqState::HyqState()
{
  // feet and base initialized to zero by default by struct Pos and Ori
  swingleg_ = false;
}

HyqState::~HyqState()
{
  // TODO Auto-generated destructor stub
}

void HyqState::ZeroVelAcc()
{
  base_.pos.v.setZero();
  base_.pos.a.setZero();
  base_.ori.v.setZero();
  base_.ori.a.setZero();
  for (hyq::LegID l : hyq::LegIDArray) {
   feet_[l].v.setZero();
   feet_[l].a.setZero();
  }
}

const LegDataMap<Eigen::Vector3d> HyqState::GetFeetPosOnly()
{
  static LegDataMap<Eigen::Vector3d> feet_pos;
  for (LegID leg : LegIDArray)
    feet_pos[leg] = feet_[leg].p;
  return feet_pos;
}

std::array<Eigen::Vector3d, kNumSides> HyqState::GetAvgSides() const
{
  typedef std::pair <Side,Side> LegSide;
  static LegDataMap<LegSide> leg_sides;

  leg_sides[LF] = LegSide( LEFT_SIDE, FRONT_SIDE);
  leg_sides[RF] = LegSide(RIGHT_SIDE, FRONT_SIDE);
  leg_sides[LH] = LegSide( LEFT_SIDE,  HIND_SIDE);
  leg_sides[RH] = LegSide(RIGHT_SIDE,  HIND_SIDE);

  std::array<Vector3d, kNumSides> pos_avg;
  for (Side s : SideArray) pos_avg[s] = Vector3d::Zero(); // zero values

  for (LegID leg : LegIDArray)
  {
    pos_avg[leg_sides[leg].first]  += feet_[leg].p;
    pos_avg[leg_sides[leg].second] += feet_[leg].p;
  }

  for (Side s : SideArray)
    pos_avg[s] /= std::tuple_size<LegSide>::value; // 2 feet per side
  return pos_avg;
}


double HyqState::GetZAvg() const
{
  std::array<Vector3d, kNumSides> avg = GetAvgSides();
  return (avg[FRONT_SIDE](Z) + avg[HIND_SIDE](Z)) / 2;
}

int HyqState::SwinglegID() const
{
  for (LegID leg : LegIDArray)
    if (swingleg_[leg])
      return leg;

  return NO_SWING_LEG;
}

void HyqState::SetSwingleg(LegID leg)
{
  swingleg_ = false;
  swingleg_[leg] = true;
}

LegDataMap<Foothold> HyqState::FeetToFootholds() const
{
  LegDataMap<Foothold> footholds;
  for (LegID leg : LegIDArray)
    footholds[leg] = FootToFoothold(leg);
  return footholds;
}

HyqState::VecFoothold
HyqState::GetStanceLegs() const
{
  VecFoothold stance_legs;
  for (LegID leg : LegIDArray)
    if (!swingleg_[leg])
      stance_legs.push_back(FootToFoothold(leg));

  return stance_legs;
}

Foothold HyqState::FootToFoothold(LegID leg) const
{
  Foothold foothold;
  foothold.p = feet_[leg].p;
  foothold.leg = leg;

  return foothold;
}

} // namespace hyq
} // namespace xpp
