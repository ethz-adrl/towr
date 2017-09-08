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
  ContactState init(n_ee_, false);

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
  SetGaits({Trot});
}

QuadrupedGaitGenerator::FootDurations
QuadrupedGaitGenerator::GetContactSchedule () const
{
  VecTimes d_accumulated(n_ee_, 0.0);

  FootDurations foot_durations(n_ee_);
  for (int phase=0; phase<contacts_.size()-1; ++phase) {

    ContactState curr = contacts_.at(phase);
    ContactState next = contacts_.at(phase+1);

    for (auto ee : curr.GetEEsOrdered()) {
      d_accumulated.at(ee) += times_.at(phase);

      // if contact will change in next phase, so this phase duration complete
      bool contacts_will_change = curr.At(ee) != next.At(ee);
      if (contacts_will_change)  {
        foot_durations.at(ee).push_back(d_accumulated.at(ee));
        d_accumulated.at(ee) = 0.0;
      }
    }
  }

  // push back last phase
  for (auto ee : contacts_.back().GetEEsOrdered())
    foot_durations.at(ee).push_back(d_accumulated.at(ee) + times_.back());


  return foot_durations;
}

void
QuadrupedGaitGenerator::SetGaits (const std::vector<QuadrupedGaits>& gaits)
{
  // initialize with stance phase
  times_    = { 0.3 };
  contacts_ = { BB_ };

  for (QuadrupedGaits g : gaits) {
    auto info = GetGait(g);
    std::vector<double>       t = info.first;
    std::vector<ContactState> c = info.second;
    assert(t.size() == c.size()); // make sure every phase has a time

    times_.insert      (times_.end(), t.begin(), t.end());
    contacts_.insert(contacts_.end(), c.begin(), c.end());

    // insert short time where all legs are in contact between gaits
//    times_.push_back(0.2);
//    contacts_.push_back(BB_);
  }

  // insert final stance phase
  times_.push_back(0.5);
  contacts_.push_back(BB_);
//  NormalizeTimesToOne();
}

QuadrupedGaitGenerator::GaiInfo
QuadrupedGaitGenerator::GetGait(QuadrupedGaits gait) const
{
  switch (gait) {
    case Stand:       return GetDurationsStand();
    case Leglift:     return GetDurationsLeglift();
    case Walk:        return GetDurationsWalk();
    case WalkOverlap: return GetDurationsWalkOverlap();
    case Trot:        return GetDurationsTrot();
    case TrotFly:     return GetDurationsTrotFly();
    case Pace:        return GetDurationsPace();
    case Bound:       return GetDurationsBound();
    case Pronk:       return GetDurationsPronk();
    default: assert(false); // gait not implemented
  }
}

// already in dynamic model
//void
//QuadrupedGaitGenerator::NormalizeTimesToOne()
//{
//  double total_time = std::accumulate(times_.begin(), times_.end(), 0.0);
//  std::transform(times_.begin(), times_.end(), times_.begin(),
//                 [total_time](double t_phase){ return t_phase/total_time;});
//}

QuadrupedGaitGenerator::GaiInfo
QuadrupedGaitGenerator::GetDurationsStand () const
{
  auto times =
  {
      1.0,
  };
  auto contacts =
  {
      BB_,
  };

  return std::make_pair(times, contacts);
}

QuadrupedGaitGenerator::GaiInfo
QuadrupedGaitGenerator::GetDurationsLeglift () const
{
  auto phase_times =
  {
      0.3,
  };
  auto phase_contacts =
  {
      Bb_,
  };

  return std::make_pair(phase_times, phase_contacts);
}

QuadrupedGaitGenerator::GaiInfo
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

QuadrupedGaitGenerator::GaiInfo
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

QuadrupedGaitGenerator::GaiInfo
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

QuadrupedGaitGenerator::GaiInfo
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

QuadrupedGaitGenerator::GaiInfo
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

QuadrupedGaitGenerator::GaiInfo
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

QuadrupedGaitGenerator::GaiInfo
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
