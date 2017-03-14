/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/endeffectors_motion.h>
#include <xpp/endeffectors4.h> // zmp_ this shouldn't be here
#include <xpp/opt/eigen_std_conversions.h>

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
    endeffectors_.At(ee).SetInitialPos(initial_pos.At(ee));
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
  for (auto ee : endeffectors_.GetEEsOrdered()) {
    int id = 1; // because initial stance is 0
    for (auto pos : endeffectors_.At(ee).GetFreeContactPositions())
      contacts.push_back(Contact(id++, ee, pos));
  }

  return contacts;
}

// zmp_ this shouldn't be here
void
EndeffectorsMotion::Set2StepTrott ()
{
  Vector3d start = Vector3d::Zero(); // initialized with this value

  // initial stance
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.4);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.4);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.4);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.4);
  // first step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.3);
  // second step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddSwingPhase(0.3, start);
  // third step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.3);
  // fourth step
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.3);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddSwingPhase(0.3, start);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddSwingPhase(0.3, start);
  // final stance
  endeffectors_.At(kMapQuadToOpt.at(LF)).AddStancePhase(0.8);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.8);
  endeffectors_.At(kMapQuadToOpt.at(LH)).AddStancePhase(0.8);
  endeffectors_.At(kMapQuadToOpt.at(RF)).AddStancePhase(0.8);
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
  StdVecEigen2d contacts;
  for (auto c : GetAllFreeContacts())
    contacts.push_back(c.p.topRows<kDim2d>()); // so far only optimizing x,y position

  return ConvertStdToEig(contacts);
}

// must be analog to the above
void
EndeffectorsMotion::SetOptimizationParameters (const VectorXd& x)
{
  int i=0;
  auto xy = ConvertEigToStd(x);
  for (auto c : GetAllFreeContacts()) {
    c.p.topRows<kDim2d>() = xy.at(i++); // changing only the xy position
    SetContactPositions({c});
  }
}

} /* namespace opt */
} /* namespace xpp */

