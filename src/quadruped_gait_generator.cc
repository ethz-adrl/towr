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
  SetDurationsTrot();
}

void
QuadrupedGaitGenerator::SetGait (QuadrupedGaits gait)
{
  switch (gait) {
    case Stand:    SetDurationsStand();break;
    case Walk:     SetDurationsWalk(); break;
    case Trot:     SetDurationsTrot(); break;
    case TrotFly:  SetDurationsTrotFly(); break;
    case Pace:     SetDurationsPace(); break;
    case Bound:    SetDurationsBound();break;
    case Pronk:    SetDurationsPronk();break;
    default: assert(false); // gait not implemented
  }
}


QuadrupedGaitGenerator::FootDurations
QuadrupedGaitGenerator::GetContactSchedule () const
{
  VecTimes d_accumulated(n_ee_, 0.0);

  FootDurations foot_durations(n_ee_);
  for (int phase=0; phase<phase_contacts_.size()-1; ++phase) {

    ContactState curr = phase_contacts_.at(phase);
    ContactState next = phase_contacts_.at(phase+1);

    for (auto ee : curr.GetEEsOrdered()) {
      d_accumulated.at(ee) += phase_times_.at(phase);

      // if contact will change in next phase, so this phase duration complete
      bool contacts_will_change = curr.At(ee) != next.At(ee);
      if (contacts_will_change)  {
        foot_durations.at(ee).push_back(d_accumulated.at(ee));
        d_accumulated.at(ee) = 0.0;
      }
    }
  }

  // push back last phase
  for (auto ee : phase_contacts_.back().GetEEsOrdered())
    foot_durations.at(ee).push_back(d_accumulated.at(ee) + phase_times_.back());


  return foot_durations;
}

void
QuadrupedGaitGenerator::SetDurationsStand ()
{
  phase_times_ =
  {
      1.0,
  };
  phase_contacts_ =
  {
      BB_,
  };
}

void
QuadrupedGaitGenerator::SetDurationsPronk ()
{
  phase_times_ =
  {
      1.0, 0.5, 1.0
  };
  phase_contacts_ =
  {
      BB_, II_, BB_
  };
}

void
QuadrupedGaitGenerator::SetDurationsWalk ()
{
  double t_phase = 0.2;
  double t_trans = 0.1;
  phase_times_ =
  {
      0.3,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      t_phase, t_trans, t_phase, t_phase, t_trans, t_phase,
      0.2,
  };
  phase_contacts_ =
  {
      BB_,
      bB_, bb_, Bb_, PB_, PP_, BP_,
      bB_, bb_, Bb_, PB_, PP_, BP_,
      BB_,
  };
}

void
QuadrupedGaitGenerator::SetDurationsTrot ()
{
  double t_phase = 0.4;
  double t_trans = 0.05;
  phase_times_ =
  {
      0.3,
      t_phase, t_trans, t_phase, t_trans,
      t_phase, t_trans, t_phase, t_trans,
      t_phase, t_trans, t_phase, t_trans,
      0.2,
  };
  phase_contacts_ =
  {
      BB_,
      bP_, BB_, Pb_, BB_,
      bP_, BB_, Pb_, BB_,
      bP_, BB_, Pb_, BB_,
      BB_,
  };
}

void
QuadrupedGaitGenerator::SetDurationsTrotFly ()
{
  double t_phase = 0.4;
  double t_trans = 0.05;
  phase_times_ =
  {
      0.3,
      t_phase, t_trans, t_phase, t_trans,
      t_phase, t_trans, t_phase, t_trans,
      t_phase, t_trans, t_phase, t_trans,
      0.2,
  };
  phase_contacts_ =
  {
      BB_,
      bP_, II_, Pb_, II_,
      bP_, II_, Pb_, II_,
      bP_, II_, Pb_, II_,
      BB_,
  };
}

void
QuadrupedGaitGenerator::SetDurationsPace ()
{
  double A = 0.4;
  double B = 0.1;
  phase_times_ =
  {
      0.3,
      A, B, A, B,
      A, B, A, B,
      A, B, A, B,
      0.2,
  };
  phase_contacts_ =
  {
      BB_,
      PP_, BB_, bb_, BB_,
      PP_, BB_, bb_, BB_,
      PP_, BB_, bb_, BB_,
      BB_,
  };
}

void
QuadrupedGaitGenerator::SetDurationsBound ()
{
  double t_phase = 0.2;
  phase_times_ =
  {
      0.3,
      t_phase, t_phase,
      t_phase, t_phase,
      t_phase, t_phase,
      0.2,
  };
  phase_contacts_ =
  {
      BB_,
      BI_, IB_,
      BI_, IB_,
      BI_, IB_,
      BB_,
  };
}

QuadrupedGaitGenerator::~QuadrupedGaitGenerator ()
{
  // TODO Auto-generated destructor stub
}

} /* namespace quad */
} /* namespace xpp */
