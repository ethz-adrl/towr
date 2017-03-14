/**
 @file    endeffectors_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 14, 2017
 @brief   Brief description
 */

#include <xpp/opt/endeffectors_motion.h>

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

} /* namespace opt */
} /* namespace xpp */
