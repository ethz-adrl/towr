/**
@file    virtual_model-inl.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Virtual model implementation to create feed-forward accelerations.

         In contrast to ioannis's virtual model implementation, this one creates
         feed-forward \b accelerations and is SL \b independent.
 */

#ifndef _IIT_VIRTUAL_MODEL_H_
#define _IIT_VIRTUAL_MODEL_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/utils/orientation.h>

#include <iit/robots/hyq/inertia_properties.h>
#include <iit/robots/hyq/transforms.h>
#include <iit/robots/hyq/jsim.h>

#include <kindr/rotations/RotationEigen.hpp>

namespace xpp {
namespace hyq {

/**
@brief Creates linear and angular virtual model accelerations based on a
       difference between a desired and a current state.
 */
class VirtualModel {
public:
  typedef iit::rbd::ForceVector Gains;
  typedef iit::rbd::VelocityVector Accelerations;
  typedef iit::HyQ::dyn::InertiaMatrix InertiaMatrix;
  typedef xpp::utils::Pose Pose;

  VirtualModel ()
  {
    Kp.setZero();
    Kd.setZero();
  }

  void SetGains()
  {
    SetGains(1000, 1000, 1000,  // position gains x,y,z -> drift    | 200, 600, 2000,
              270,  270,  270,  // velocity gains x,y,z             | 200, 200, 400,
             1000, 1000,  500,  // position gains roll, pitch, yaw  | 1000, 1000, 1000
              100,  100,  100); // velocity gains roll, pitch, yaw  | 100, 100, 100);
  }

  void SetGains(double lx_p, double ly_p, double lz_p,
                double lx_d, double ly_d, double lz_d,
                double ax_p, double ay_p, double az_p,
                double ax_d, double ay_d, double az_d)
  {
    using namespace xpp::utils;
    Kp(LX) = lx_p; Kp(LY) = ly_p; Kp(LZ) = lz_p;
    Kd(LX) = lx_d; Kd(LY) = ly_d; Kd(LZ) = lz_d;

    Kp(AX) = ax_p; Kp(AY) = ay_p; Kp(AZ) = az_p;
    Kd(AX) = ax_d; Kd(AY) = ay_d; Kd(AZ) = az_d;
  }


  /**
   * @brief Calculates necessary body accelerations to move the body from curr
   *        to des.
   * @param curr Current position and orientation from state estimation.
   * @param des Desired position from motion plan.
   * @param I Inertia matrix for angular and linear (only robot mass) acceleration.
   * @param base_fb_acc Required feed-back acceleration of the robot base.
   */
  void CalcFeedbackAcc(const Pose& i_curr, const Pose& i_des,
                       const InertiaMatrix& I,
                       Accelerations& i_base_fb_acc) const;

private:
  Gains Kp;
  Gains Kd;
};


inline void
VirtualModel::CalcFeedbackAcc(const Pose& i_curr, const Pose& i_des,
                              const InertiaMatrix& intertia_matrix,
                              Accelerations& i_base_fb_acc) const
{
  using namespace xpp::utils; // X, Y, Z, LX, AX, ...

  //these generalized moments and forces are in the world frame
  iit::rbd::ForceVector  i_base_wrench;

  kindr::rotations::eigen_impl::RotationQuaternionPD I_q_Bcurr(i_curr.ori.q);
  kindr::rotations::eigen_impl::RotationQuaternionPD I_q_Bdes(i_des.ori.q);
  kindr::rotations::eigen_impl::EulerAnglesXyzPD Bcurr_rpy_Bdes(I_q_Bcurr.inverted()*I_q_Bdes);
  Bcurr_rpy_Bdes.setUnique();

  // Position control on body
  i_base_wrench(LX)  = Kp(LX) * (i_des.pos.p(X) - i_curr.pos.p(X));
  i_base_wrench(LY)  = Kp(LY) * (i_des.pos.p(Y) - i_curr.pos.p(Y));
  i_base_wrench(LZ)  = Kp(LZ) * (i_des.pos.p(Z) - i_curr.pos.p(Z));

  i_base_wrench(AX)  = Kp(AX) * Bcurr_rpy_Bdes.roll();
  i_base_wrench(AY)  = Kp(AY) * Bcurr_rpy_Bdes.pitch();
  i_base_wrench(AZ)  = Kp(AZ) * Bcurr_rpy_Bdes.yaw();

  // velocity control on body
  i_base_wrench(LX) += Kd(LX) * (i_des.pos.v(X) - i_curr.pos.v(X));
  i_base_wrench(LY) += Kd(LY) * (i_des.pos.v(Y) - i_curr.pos.v(Y));
  i_base_wrench(LZ) += Kd(LZ) * (i_des.pos.v(Z) - i_curr.pos.v(Z));

  i_base_wrench(AX) += Kd(AX) * (i_des.ori.v(X) - i_curr.ori.v(X));
  i_base_wrench(AY) += Kd(AY) * (i_des.ori.v(Y) - i_curr.ori.v(Y));
  i_base_wrench(AZ) += Kd(AZ) * (i_des.ori.v(Z) - i_curr.ori.v(Z));


  Eigen::Matrix3d i_R_b = i_curr.ori.q.toRotationMatrix();
  Eigen::Vector3d b_base_torque = i_R_b.transpose()*i_base_wrench.segment(AX, 3);

  Eigen::Matrix3d I = intertia_matrix.get3x3Tensor();
  double mass_robot = intertia_matrix(LX, LX); // also LY or LZ work
  // alpha = I^(-1) * M
  i_base_fb_acc.segment(AX, 3) = i_R_b * (I.inverse()*b_base_torque);
  // a = F / m
  i_base_fb_acc.segment(LX, 3) = 1./mass_robot * i_base_wrench.segment(LX, 3);
}

} // namespace xpp
} // namespace hyq

#endif /* IIT_VIRTUAL_MODEL_H_ */
