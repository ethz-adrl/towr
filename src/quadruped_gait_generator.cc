/**
 @file    quadruped_gait_generator.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 4, 2017
 @brief   Brief description
 */

#include <xpp/quadruped_gait_generator.h>

namespace xpp {
namespace quad {

QuadrupedGaitGenerator::QuadrupedGaitGenerator ()
{
  int n_ee = 4;
  ContactState init(n_ee, false);

  II_                               = init; // flight_phase
  PI_ = bI_ = IP_ = Ib_             = init; // single contact
  Pb_ = bP_ = BI_ = IB_ = PP_ = bb_ = init; // two leg support
  Bb_ = BP_ = bB_ = PB_             = init; // three-leg support
  BB_                               = init; // four-leg support phase


  auto m = kMapIDToEE; // shorthand for readability

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
  SetGaits({opt::Stand});
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetGait(opt::GaitTypes gait) const
{
  switch (gait) {
    case opt::Stand:   return GetDurationsStand();
    case opt::Flight:  return GetDurationsFlight();
    case opt::Walk1:   return GetDurationsWalk();
    case opt::Walk2:   return GetDurationsWalkOverlap();
    case opt::Run1:    return GetDurationsTrot();
    case opt::Run2:    return GetDurationsTrotFly();
    case opt::Run3:    return GetDurationsPace();
    case opt::Hop1:    return GetDurationsBound();
    case opt::Hop2:    return GetDurationsPronk();
    default: assert(false); // gait not implemented
  }
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsStand () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      BB_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsFlight () const
{
  auto times =
  {
      0.5,
  };
  auto contacts =
  {
      II_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsPronk () const
{
  auto times =
  {
      0.1, 0.3,
      0.1, 0.3,
      0.1, 0.3,
  };
  auto phase_contacts =
  {
      II_, BB_,
      II_, BB_,
      II_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsWalk () const
{
  double t_phase = 0.3;
  double t_trans = 0.13;
  auto times =
  {
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
  };
  auto phase_contacts =
  {
      bB_, bb_, Bb_, PB_, PP_, BP_,
      bB_, bb_, Bb_, PB_, PP_, BP_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsWalkOverlap () const
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
      // next stride
      three, lateral, three,
      diagonal,
      three, lateral, three,
      diagonal,
  };
  auto phase_contacts =
  {
      bB_, bb_, Bb_,
      Pb_,
      PB_, PP_, BP_,
      bP_,
      // next stride
      bB_, bb_, Bb_,
      Pb_,
      PB_, PP_, BP_,
      bP_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsTrot () const
{
  double t_phase = 0.4;
  double t_trans = 0.2;
  auto times =
  {
      t_phase, t_trans, t_phase, t_trans,
      t_phase, t_trans, t_phase, t_trans,
      t_phase, t_trans, t_phase, t_trans,
  };
  auto phase_contacts =
  {
      bP_, BB_, Pb_, BB_,
      bP_, BB_, Pb_, BB_,
      bP_, BB_, Pb_, BB_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsTrotFly () const
{
  double stance = 0.3;
  double flight = 0.05;
  auto times =
  {
      stance, flight, stance, flight,
      stance, flight, stance, flight,
      stance, flight, stance, flight,
  };
  auto phase_contacts =
  {
      bP_, II_, Pb_, II_,
      bP_, II_, Pb_, II_,
      bP_, II_, Pb_, II_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsPace () const
{
  double A = 0.3;
  double B = 0.05;
  auto times =
  {
      A, B, A, B,
      A, B, A, B,
      A, B, A, B,
  };
  auto phase_contacts =
  {
      PP_, II_, bb_, II_,
      PP_, II_, bb_, II_,
      PP_, II_, bb_, II_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::GaitInfo
QuadrupedGaitGenerator::GetDurationsBound () const
{
  double A = 0.3;
  double B = 0.05;
  auto times =
  {
      A, B, A, B,
      A, B, A, B,
      A, B, A, B,
  };
  auto phase_contacts =
  {
      BI_, II_, IB_, II_,
      BI_, II_, IB_, II_,
      BI_, II_, IB_, II_,
  };

  return std::make_pair(times, phase_contacts);
}

QuadrupedGaitGenerator::~QuadrupedGaitGenerator ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace quad */
} /* namespace xpp */
