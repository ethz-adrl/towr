/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/endeffectors_motion.h>
#include <xpp/endeffectors4.h> // zmp_ this shouldn't be here

namespace xpp {
namespace opt {

EndeffectorsMotion::EndeffectorsMotion ()
{
//  endeffectors_.SetCount(n_ee);
}

EndeffectorsMotion::~EndeffectorsMotion ()
{
  // TODO Auto-generated destructor stub
}

void
EndeffectorsMotion::SetInitialPos (const EEXppPos& initial_pos)
{
  endeffectors_.SetCount(initial_pos.GetEECount());

  for (auto ee : initial_pos.GetEEsOrdered())
    endeffectors_.At(ee).SetInitialPos(initial_pos.At(ee), ee);
}

EEMotion&
EndeffectorsMotion::GetMotion (EndeffectorID ee)
{
  return endeffectors_.At(ee);
}

EndeffectorsMotion::EEState
EndeffectorsMotion::GetEndeffectors (double t_global) const
{
  EEState ee_state(endeffectors_.GetEECount());

  for (auto ee : endeffectors_.GetEEsOrdered())
    ee_state.At(ee) = endeffectors_.At(ee).GetState(t_global);

  return ee_state;
}

EndeffectorsMotion::Contacts
EndeffectorsMotion::GetAllFreeContacts () const
{
  int idx = 0; // zmp_ function doing two things, ugly
  Contacts contacts;
  for (auto ee : endeffectors_.ToImpl()) {
    map_ee_to_first_step_idx_[ee.ee_] = idx;
    for (auto c : ee.GetFreeContacts()) {
      contacts.push_back(c);
      idx++;
    }
  }

  return contacts;
}

EndeffectorsMotion::Contacts
EndeffectorsMotion::GetContacts (double t) const
{
  Contacts contacts;
  for (auto ee : endeffectors_.ToImpl())
    for (auto c : ee.GetContact(t)) // can be one or none (if swinging)
      contacts.push_back(c);

  return contacts;
}

EEXppBool
EndeffectorsMotion::GetContactState (double t_global) const
{
  EEXppBool contact_state(endeffectors_.GetEECount());

  for (auto ee : endeffectors_.ToImpl())
    contact_state.At(ee.ee_) = ee.IsInContact(t_global);

  return contact_state;
}

void
EndeffectorsMotion::SetContactPositions (const Contacts& contacts)
{
  for (auto c : contacts)
    endeffectors_.At(c.ee).SetContactPosition(c.id,c.p);
}

EndeffectorsMotion::VectorXd
EndeffectorsMotion::GetOptimizationParameters () const
{
  VectorXd x(GetAllFreeContacts().size() * kDim2d);
  for (auto c : GetAllFreeContacts())
    for (auto dim : {X,Y})
      x(Index(c.ee, c.id, dim)) = c.p(dim);

  return x;
}

// must be analog to the above
void
EndeffectorsMotion::SetOptimizationParameters (const VectorXd& x)
{
  for (auto c : GetAllFreeContacts()) {
    for (auto dim : {X,Y})
      c.p(dim) = x(Index(c.ee, c.id, dim));

    SetContactPositions({c});
  }
}

int
EndeffectorsMotion::Index (EndeffectorID ee, int id, Coords3D coords3D) const
{
  // the position of this contact in the overall vector
  int id_total = map_ee_to_first_step_idx_.at(ee) + (id-1);
  return kDim2d*id_total + coords3D;
}

bool
EndeffectorsMotion::Contains (const EEVec& v, EndeffectorID ee) const
{
  return std::find(v.begin(), v.end(), ee) != v.end();
}

double
EndeffectorsMotion::GetTotalTime () const
{
   return endeffectors_.At(E0).GetTotalTime();
}

EndeffectorsMotion::EEVec
EndeffectorsMotion::GetStanceLegs (const EEVec& swinglegs) const
{
  EEVec stance_legs;
  for (auto ee : endeffectors_.GetEEsOrdered())
    if (!Contains(swinglegs, ee)) // endeffector currently in stance phase
      stance_legs.push_back(ee);

  return stance_legs;
}



void
EndeffectorsMotion::SetPhaseSequence (const PhaseVec& phases)
{
  Vector3d start = Vector3d::Zero(); // initialized with this value

  Endeffectors<double> durations(endeffectors_.GetEECount());
  durations.SetAll(0.0);


  for (int i=0; i<phases.size()-1; ++i) {

    EEVec swinglegs       = phases.at(i).first;
    EEVec next_swinglegs  = phases.at(i+1).first;
    double phase_duration = phases.at(i).second;

    // stance phases
    for (auto ee : GetStanceLegs(swinglegs)) {
      durations.At(ee) += phase_duration;
      if(Contains(next_swinglegs,ee)) {  // leg swingwing in next phase or last phase
        endeffectors_.At(ee).AddStancePhase(durations.At(ee));
        durations.At(ee) = 0.0; // reset
      }
    }

    // swing phases
    for (auto ee : swinglegs) {
      durations.At(ee) += phase_duration;
      if(!Contains(next_swinglegs, ee)) {  //next swinglegs do not contain endeffector
        endeffectors_.At(ee).AddSwingPhase(durations.At(ee), start);
        durations.At(ee) = 0.0; // reset
      }
    }
  }


  // last phase always must be added
  EEVec swinglegs = phases.back().first;
  double T        = phases.back().second;
  for (auto ee : swinglegs)
    endeffectors_.At(ee).AddSwingPhase(durations.At(ee) + T, start);
  for (auto ee : GetStanceLegs(swinglegs))
    endeffectors_.At(ee).AddStancePhase(durations.At(ee)+T);

}

// zmp_ this shouldn't be here
void
EndeffectorsMotion::Set2StepTrott ()
{
  Vector3d start = Vector3d::Zero(); // initialized with this value

  // initial stance
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.4);
  endeffectors_.At(kMapQuadToOpt.at(RH)).AddStancePhase(0.4);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.7);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.7);
  // first step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RH)).AddSwingPhase(0.3, start);
//  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.3);
//  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.3);
  // second step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(RH)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddSwingPhase(0.3, start);

  // third step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RH)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.3);
  // fourth step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(1.1);
  endeffectors_.At(kMapQuadToOpt.at(RH)).AddStancePhase(1.1);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddSwingPhase(0.3, start);
  // final stance of 0.8s
//  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.8);
//  endeffectors_.At(kMapQuadToOpt.at(RH)).AddStancePhase(0.8);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.8);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.8);
}

} /* namespace opt */
} /* namespace xpp */

