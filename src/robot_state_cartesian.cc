/**
 @file    articulated_robot_state_cartesian.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2017
 @brief   Brief description
 */

#include <xpp/robot_state_cartesian.h>

namespace xpp {

RobotStateCommon::RobotStateCommon (int n_ee)
    :is_contact_(n_ee)
{
  is_contact_.SetAll(true);
  t_global_ = 0.0;
}

RobotStateCommon::~RobotStateCommon ()
{
}


RobotDataHolder::RobotDataHolder (int n_ee)
    :data_(n_ee)
{
}

RobotDataHolder::~RobotDataHolder ()
{
}

RobotStateCartesian::RobotStateCartesian (int n_ee)
    :RobotDataHolder(n_ee),
     feet_W_(n_ee)
{
}

RobotStateCartesian::~RobotStateCartesian ()
{
  // TODO Auto-generated destructor stub
}

void
RobotStateCartesian::SetEEStateInWorld (MotionDerivative dxdt, const EEPos& val)
{
  feet_W_.SetCount(val.GetCount());
  for (EEID ee : GetEndeffectors())
    feet_W_.At(ee).GetByIndex(dxdt) = val.At(ee);
}

void
RobotStateCartesian::SetEEStateInWorld (const FeetArray& ee)
{
  feet_W_ = ee;
}

const RobotStateCartesian::FeetArray&
RobotStateCartesian::GetEEState () const
{
  return feet_W_;
}

void
RobotStateCartesian::SetEEForcesInWorld (const EEForces& val)
{
  ee_forces_ = val;
}

RobotStateCartesian::EEForces
RobotStateCartesian::GetEEForces () const
{
  return ee_forces_;
}

double
RobotStateCartesian::GetZAvg () const
{
  double z_avg = 0.0;
  for (EEID ee : GetEndeffectors())
    if (GetContactState().At(ee))
      z_avg += (feet_W_.At(ee).p_.z()/GetEEInContactCount());

  return z_avg;
}

RobotStateCartesian::EEPos
RobotStateCartesian::GetEEPos () const
{
  EEPos pos_W(GetEECount());
  for (EEID ee : GetEndeffectors())
    pos_W.At(ee) = feet_W_.At(ee).p_;

  return pos_W;
}

} /* namespace xpp */

