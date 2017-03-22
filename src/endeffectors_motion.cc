/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/endeffectors_motion.h>

namespace xpp {
namespace opt {

EndeffectorsMotion::EndeffectorsMotion (int n_ee)
{
  endeffectors_.SetCount(n_ee);
}

EndeffectorsMotion::~EndeffectorsMotion ()
{
}

void
EndeffectorsMotion::SetInitialPos (const EEXppPos& initial_pos)
{
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
  Contacts contacts;
  for (auto ee : endeffectors_.ToImpl())
    for (auto c : ee.GetFreeContacts())
      contacts.push_back(c);

  return contacts;
}

EndeffectorsMotion::Contacts
EndeffectorsMotion::GetContacts (double t) const
{
  Contacts contacts;
  for (auto ee : endeffectors_.ToImpl())
    for (Contact c : ee.GetContact(t)) // can be one or none (if swinging)
      contacts.push_back(c);

  return contacts;
}

EEXppBool
EndeffectorsMotion::GetContactState (double t_global) const
{
  EEXppBool contact_state(endeffectors_.GetEECount());

  for (auto ee : endeffectors_.ToImpl())
    contact_state.At(ee.GetEE()) = ee.IsInContact(t_global);

  return contact_state;
}

void
EndeffectorsMotion::SetContactPositions (const Contacts& contacts)
{
  for (auto c : contacts)
    endeffectors_.At(c.ee).UpdateContactPosition(c.id,c.p);
}

EndeffectorsMotion::VectorXd
EndeffectorsMotion::GetOptimizationParameters () const
{
  VectorXd x(GetAllFreeContacts().size() * kDim2d);
  for (auto c : GetAllFreeContacts())
    for (auto dim : d2::AllDimensions)
      x(Index(c.ee, c.id, dim)) = c.p(dim);

  return x;
}

// must be analog to the above
void
EndeffectorsMotion::SetOptimizationParameters (const VectorXd& x)
{
  for (auto c : GetAllFreeContacts()) {
    for (auto dim : d2::AllDimensions)
      c.p(dim) = x(Index(c.ee, c.id, dim));

    SetContactPositions({c});
  }
}

int
EndeffectorsMotion::GetOptVarCount () const
{
  // zmp_ DRY can be summarized in base class
  return GetOptimizationParameters().rows();
}

int
EndeffectorsMotion::Index (EndeffectorID _ee, int id, d2::Coords dimension) const
{
  // stored like this in vector, initial contacts not optimized over
  // (E0_0), E0_1, E0_2, ...
  // (E1_0), E1_1, E1_1, ...
  // (E2_0), E2_1, E2_1, ...

  int position_in_vector = id-1; // -1 because first contact not optimized over

  for (auto ee : endeffectors_.GetEEsOrdered()) {
    if (ee == _ee) break;
    position_in_vector +=  endeffectors_.At(ee).GetFreeContacts().size();
  }

  return position_in_vector*kDim2d + dimension;
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
      if(Contains(next_swinglegs,ee)) {  // leg swingwing in next phase
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


  std::cout << "contacts: " << std::endl;
  for (auto ee : endeffectors_.ToImpl()) {
    for (auto c :ee.GetFreeContacts())
      std::cout << c << std::endl;
    std::cout << std::endl;
  }
}

} /* namespace opt */
} /* namespace xpp */
