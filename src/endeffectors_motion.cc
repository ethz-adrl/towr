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
    :Parametrization("footholds")
{
  endeffectors_.SetCount(n_ee);
}

EndeffectorsMotion::~EndeffectorsMotion ()
{
}

void
EndeffectorsMotion::SetInitialPos (const EndeffectorsPos& initial_pos)
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
  EEState ee_state(GetNumberOfEndeffectors());

  for (auto ee : endeffectors_.GetEEsOrdered())
    ee_state.At(ee) = endeffectors_.At(ee).GetState(t_global);

  return ee_state;
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

EndeffectorsBool
EndeffectorsMotion::GetContactState (double t_global) const
{
  EndeffectorsBool contact_state(GetNumberOfEndeffectors());

  for (auto ee : endeffectors_.GetEEsOrdered())
    contact_state.At(ee) = endeffectors_.At(ee).IsInContact(t_global);

  return contact_state;
}

EndeffectorsMotion::VectorXd
EndeffectorsMotion::GetOptimizationParameters () const
{
  VectorXd x(n_opt_params_);

  int row = 0;
  for (const auto& ee : endeffectors_.ToImpl()) {
    int n = ee.GetOptVarCount();
    x.middleRows(row, n) = ee.GetOptimizationParameters();
    row += n;
  }

  return x;
}

// must be analog to the above
void
EndeffectorsMotion::SetOptimizationParameters (const VectorXd& x)
{
  int row = 0;

  for (auto ee : endeffectors_.GetEEsOrdered()) {
    int n = endeffectors_.At(ee).GetOptVarCount();
    endeffectors_.At(ee).SetOptimizationParameters(x.middleRows(row, n));
    row += n;
  }
}

int
EndeffectorsMotion::Index (EndeffectorID ee, int id, d2::Coords dimension) const
{
  int idx = 0;

  for (auto _ee : endeffectors_.GetEEsOrdered()) {
    auto motion = endeffectors_.At(_ee);
    if (_ee == ee)
      return idx + motion.Index(id, dimension);

    idx += motion.GetOptVarCount();
  }

  assert(false); // _ee does not exist
}

double
EndeffectorsMotion::GetTotalTime () const
{
   return endeffectors_.At(E0).GetTotalTime();
}

int
EndeffectorsMotion::GetNumberOfEndeffectors () const
{
  return endeffectors_.GetCount();
}

void
EndeffectorsMotion::SetPhaseSequence (const PhaseVec& phases)
{
  Vector3d start = Vector3d::Zero(); // initialized with this value

  Endeffectors<double> durations(GetNumberOfEndeffectors());
  durations.SetAll(0.0);

  for (int i=0; i<phases.size()-1; ++i) {

    EndeffectorsBool is_swingleg      = phases.at(i).first;
    EndeffectorsBool is_swingleg_next = phases.at(i+1).first;
    double phase_duration             = phases.at(i).second;

    for (auto ee : is_swingleg.GetEEsOrdered()) {

      durations.At(ee) += phase_duration;

      // check if next phase is different phase
      bool next_different = is_swingleg.At(ee) != is_swingleg_next.At(ee);

      if (next_different) {

        if (!is_swingleg.At(ee)) { // stance leg to swing
          endeffectors_.At(ee).AddStancePhase(durations.At(ee));
          durations.At(ee) = 0.0; // reset

        } else { // swinglegleg to stance
          endeffectors_.At(ee).AddSwingPhase(durations.At(ee), start);
          durations.At(ee) = 0.0; // reset
        }
      }
    }
  }

  EndeffectorsBool swinglegs = phases.back().first;
  double T                   = phases.back().second;

  for (auto ee : swinglegs.GetEEsOrdered()) {
    if (!swinglegs.At(ee)) // last phase is stance
      endeffectors_.At(ee).AddStancePhase(durations.At(ee)+T);
    else
      endeffectors_.At(ee).AddSwingPhase(durations.At(ee) + T, start);
  }

  // count number of optimization variables
  n_opt_params_ = 0;
  for (const auto& ee : endeffectors_.ToImpl()) {
    n_opt_params_ += ee.GetOptVarCount();
  }

}

} /* namespace opt */
} /* namespace xpp */
