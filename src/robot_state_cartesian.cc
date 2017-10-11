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

Endeffectors<Vector3d>
RobotStateCartesian::GetEEVelInBase () const
{
  using namespace Eigen;
  Endeffectors<Vector3d> vel_B(ee_motion_.GetCount());
  Vector3d base_ang_W = base_.ang.v;
  Vector3d base_lin_W = base_.lin.v_;
  Matrix3d B_R_W = base_.ang.q.normalized().toRotationMatrix().transpose();

  for (auto ee : ee_motion_.GetEEsOrdered()) {

    Vector3d pos_ee_W = ee_motion_.At(ee).p_;
    Vector3d vel_ee_W = ee_motion_.At(ee).v_;

    Vector3d rel_vel_ee_W = vel_ee_W - base_lin_W + base_ang_W.cross(pos_ee_W);
    vel_B.At(ee) = B_R_W * rel_vel_ee_W;
  }

  return vel_B;
}

} /* namespace xpp */

