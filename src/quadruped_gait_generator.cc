/**
 @file    quadruped_gait_generator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 4, 2017
 @brief   Brief description
 */

#include <xpp/models/quadruped_gait_generator.h>

namespace xpp {
namespace opt {

QuadrupedGaitGenerator::QuadrupedGaitGenerator ()
{
  int n_ee = 4;
  ContactState init(n_ee, false);

  II_                               = init; // flight_phase
  PI_ = bI_ = IP_ = Ib_             = init; // single contact
  Pb_ = bP_ = BI_ = IB_ = PP_ = bb_ = init; // two leg support
  Bb_ = BP_ = bB_ = PB_             = init; // three-leg support
  BB_                               = init; // four-leg support phase


  using namespace quad;
  map_id_to_ee_ = quad::kMapIDToEE;
  auto m = map_id_to_ee_; // shorthand for readability

  // flight_phase
  II_.SetAll(false);
  // one stanceleg
  PI_.At(m.at(LH)) = true;
  bI_.At(m.at(RH)) = true;
  IP_.At(m.at(LF)) = true;
  Ib_.At(m.at(RF)) = true;
  // two stancelegs
  Pb_.At(m.at(LH)) = true; Pb_.At(m.at(RF)) = true;
  bP_.At(m.at(RH)) = true; bP_.At(m.at(LF)) = true;
  BI_.At(m.at(LH)) = true; BI_.At(m.at(RH)) = true;
  IB_.At(m.at(LF)) = true; IB_.At(m.at(RF)) = true;
  PP_.At(m.at(LH)) = true; PP_.At(m.at(LF)) = true;
  bb_.At(m.at(RH)) = true; bb_.At(m.at(RF)) = true;
  // three stancelegs
  Bb_.At(m.at(LH)) = true; Bb_.At(m.at(RH)) = true;  Bb_.At(m.at(RF))= true;
  BP_.At(m.at(LH)) = true; BP_.At(m.at(RH)) = true;  BP_.At(m.at(LF))= true;
  bB_.At(m.at(RH)) = true; bB_.At(m.at(LF)) = true;  bB_.At(m.at(RF))= true;
  PB_.At(m.at(LH)) = true; PB_.At(m.at(LF)) = true;  PB_.At(m.at(RF))= true;
  // four stancelgs
  BB_.SetAll(true);

  // default gait
  SetGaits({Stand});
}

void
QuadrupedGaitGenerator::SetCombo (GaitCombos combo)
{
  switch (combo) {
    case Combo0: SetGaits({Stand, Walk2, Walk2, Walk3E, Stand}); break;
    case Combo1: SetGaits({Stand, Run2, Run2, Run2E, Stand}); break;
    case Combo2: SetGaits({Stand, Hop5, Hop5, Hop5E, Stand}); break;
    case Combo3: SetGaits({Stand, Hop3, Hop3, Hop3E, Stand}); break;
    default: assert(false); break; // gait combo not implemented
  }
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetGait(GaitTypes gait) const
{
  switch (gait) {
    case Stand:   return GetStrideStand();
    case Flight:  return GetStrideFlight();
    case Walk1:   return GetStrideWalk();
    case Walk2:   return GetStrideWalkOverlap();
    case Walk3E:  return RemoveTransition(GetStrideWalkOverlap());
    case Run1:    return GetStrideTrot();
    case Run2:    return GetStrideTrotFly();
    case Run2E:   return RemoveTransition(GetStrideTrotFly());
    case Run3:    return GetStridePace();
    case Hop1:    return GetStrideBound();
    case Hop2:    return GetStridePronk();
    case Hop3:    return GetStrideGallop();
    case Hop3E:   return RemoveTransition(GetStrideGallop());
    case Hop5:    return GetStrideFlyingGallop();
    case Hop5E:   return RemoveTransition(GetStrideFlyingGallop());
    default: assert(false); // gait not implemented
  }
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideStand () const
{
  auto times =
  {
      0.3,
  };
  auto contacts =
  {
      BB_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideFlight () const
{
  auto times =
  {
      0.3,
  };
  auto contacts =
  {
      II_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStridePronk () const
{
  auto times =
  {
      0.1, 0.3,
  };
  auto phase_contacts =
  {
      II_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideWalk () const
{
  double step  = 0.3;
  double stand = 0.1;
  auto times =
  {
      step, stand, step, stand,
      step, stand, step, stand,
  };
  auto phase_contacts =
  {
      bB_, BB_, Bb_, BB_,
      PB_, BB_, BP_, BB_
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideWalkOverlap () const
{
  double three    = 0.3;
  double lateral  = 0.13;
  double diagonal = 0.13;

  auto times =
  {
      three, lateral, three,
      diagonal,
      three, lateral, three,
      diagonal,
  };
  auto phase_contacts =
  {
      bB_, bb_, Bb_,
      Pb_, // starting lifting RH
      PB_, PP_, BP_,
      bP_, // start lifting LH
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideTrot () const
{
  double t_phase = 0.4;
  double t_trans = 0.2;
  auto times =
  {
      t_phase, t_trans, t_phase, t_trans,
  };
  auto phase_contacts =
  {
      bP_, BB_, Pb_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideTrotFly () const
{
  double stance = 0.3;
  double flight = 0.1;
  auto times =
  {
      stance, flight, stance, flight,
  };
  auto phase_contacts =
  {
      bP_, II_, Pb_, II_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStridePace () const
{
  double A = 0.3;
  double B = 0.4;
  auto times =
  {
      A, B, A, B,
  };
  auto phase_contacts =
  {
      PP_, BB_, bb_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideBound () const
{
  double A = 0.3;
  double B = 0.3;
  auto times =
  {
      A, B, A, B,
  };
  auto phase_contacts =
  {
      BI_, BB_, IB_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideGallop () const
{
  double A = 0.3; // both feet in air
  double B = 0.2; // overlap
  double C = 0.2; // transition front->hind
  auto times =
  {
      B, A, B,
      C,
      B, A, B,
      C
  };
  auto phase_contacts =
  {
      Bb_, BI_, BP_,  // front legs swing forward
      bP_,            // transition phase
      bB_, IB_, PB_,  // hind legs swing forward
      Pb_
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetStrideFlyingGallop () const
{
  double A = 0.3; // both feet in air
  double B = 0.1; // overlap
  auto times =
  {
      // from stelians paper
      0.1, 0.1, 0.1,
      0.3, 0.2, 0.2, 0.1
  };
  auto phase_contacts =
  {
      // from stelians paper
      PI_, Pb_, Ib_,
      IP_, II_, bI_, BI_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::~QuadrupedGaitGenerator ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace opt */
} /* namespace xpp */


