/**
 @file    articulated_robot_state_cartesian.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 10, 2017
 @brief   Brief description
 */

#include <xpp/robot_state_cartesian.h>

namespace xpp {

RobotStateCartesian::RobotStateCartesian(int n_ee)
{
  ee_motion_.SetCount(n_ee);
  ee_forces_.SetCount(n_ee);
  ee_contact_.SetCount(n_ee);
  ee_contact_.SetAll(true);
  t_global_ = 0.0;
};


Endeffectors<Vector3d>
RobotStateCartesian::GetEEPos () const
{
  Endeffectors<Vector3d> pos_W(ee_motion_.GetCount());
  for (auto ee : ee_motion_.GetEEsOrdered())
    pos_W.At(ee) = ee_motion_.At(ee).p_;

  return pos_W;
}

Endeffectors<Vector3d>
RobotStateCartesian::GetEEAcc () const
{
  Endeffectors<Vector3d> acc_W(ee_motion_.GetCount());
  for (auto ee : ee_motion_.GetEEsOrdered())
    acc_W.At(ee) = ee_motion_.At(ee).a_;

  return acc_W;
}

} /* namespace xpp */

