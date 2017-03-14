/**
 @file    ee_height_z_polynomial.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 16, 2017
 @brief   Brief description
 */

#include <xpp/opt/ee_swing_motion.h>

namespace xpp {
namespace opt {

EESwingMotion::EESwingMotion ()
{
  duration_ = 0.0;
}

EESwingMotion::~EESwingMotion ()
{
  // TODO Auto-generated destructor stub
}

//void
//EESwingMotion::SetZParams (double p, double z_max)
//{
////  double t_z_total = duration_/(1-p);
//  // Setting 6 constraints:
//  // initial pos/vel is zero
//  // pos,vel at halftime is z_max and zero
//  // pos,vel at t_step is zero
//  // see matlab script "swingleg_z_height.m" for generation of these values
//
////  poly_z_.c[Polynomial::A] = 0;
////  poly_z_.c[Polynomial::B] =  16*z_max /std::pow(t_z_total,4);
////  poly_z_.c[Polynomial::C] = -32*z_max /std::pow(t_z_total,3);
////  poly_z_.c[Polynomial::D] =  16*z_max /std::pow(t_z_total,2);
////  poly_z_.c[Polynomial::E] = 0;
////  poly_z_.c[Polynomial::F] = 0;
////  poly_z_.duration = t_z_total;
//
//
////  t_start_z_ = t_z_total - duration_;
//
////  double h = 0.03; //m
////  int n=6; // match height h at 1/9*t_step and 8/9*step
////  double t_step = duration_;
////  double z_start = 0.0;
////  double z_end = 0.0; // zmp_ set these from footholds
////
////  double n2 = std::pow(n,2);
////  double n3 = std::pow(n,3);
////  double n4 = std::pow(n,4);
////  double n5 = std::pow(n,5);
////
////  double t2 = std::pow(t_step,2);
////  double t3 = std::pow(t_step,3);
////  double t4 = std::pow(t_step,4);
////  double t5 = std::pow(t_step,5);
////
////  // see matlab script "swingleg_z_height.m" for generation of these values
////  poly_z_.c[Polynomial::A] = -(2*(2*n2*z_end - 3*n3*z_end - 2*n2*z_start + 3*n3*z_start))/(t5*(n - 2)*(n2 - 2*n + 1));
////  poly_z_.c[Polynomial::B] = -(2*h*n4 - h*n5 - 10*n2*z_end + 15*n3*z_end + 10*n2*z_start - 15*n3*z_start)/(t4*(n - 2)*(n2 - 2*n + 1));
////  poly_z_.c[Polynomial::C] =  (2*(2*z_end - 2*z_start - 5*n*z_end + 5*n*z_start + 2*h*n4 - h*n5 + 5*n3*z_end - 5*n3*z_start))/((n - 2)*(n2*t3 - 2*n*t3 + t3));
////  poly_z_.c[Polynomial::D] =  (6*z_end - 6*z_start - 15*n*z_end + 15*n*z_start + 2*h*n4 - h*n5 + 10*n2*z_end - 10*n2*z_start)/(- n3*t2 + 4*n2*t2 - 5*n*t2 + 2*t2);
////  poly_z_.c[Polynomial::E] =  0;
////  poly_z_.c[Polynomial::F] =  z_start;
////  poly_z_.duration = t_z_total;
//}

void
EESwingMotion::SetContacts (const Vector3d& start_pos, const Vector3d& end_pos)
{
  StateLin3d start_state(start_pos);
  StateLin3d end_state(end_pos);

  poly_xy_.SetBoundary(duration_, start_state.Get2D(), end_state.Get2D());



  // zmp_ clean this up
  double h = 0.03; //m
  int n=6; // match height h at 1/9*t_step and 8/9*step
  double t_step = duration_;
  double z_start = start_pos.z();
  double z_end = end_pos.z();

  double n2 = std::pow(n,2);
  double n3 = std::pow(n,3);
  double n4 = std::pow(n,4);
  double n5 = std::pow(n,5);

  double t2 = std::pow(t_step,2);
  double t3 = std::pow(t_step,3);
  double t4 = std::pow(t_step,4);
  double t5 = std::pow(t_step,5);

  // see matlab script "swingleg_z_height.m" for generation of these values
  poly_z_.c[Polynomial::A] = -(2*(2*n2*z_end - 3*n3*z_end - 2*n2*z_start + 3*n3*z_start))/(t5*(n - 2)*(n2 - 2*n + 1));
  poly_z_.c[Polynomial::B] = -(2*h*n4 - h*n5 - 10*n2*z_end + 15*n3*z_end + 10*n2*z_start - 15*n3*z_start)/(t4*(n - 2)*(n2 - 2*n + 1));
  poly_z_.c[Polynomial::C] =  (2*(2*z_end - 2*z_start - 5*n*z_end + 5*n*z_start + 2*h*n4 - h*n5 + 5*n3*z_end - 5*n3*z_start))/((n - 2)*(n2*t3 - 2*n*t3 + t3));
  poly_z_.c[Polynomial::D] =  (6*z_end - 6*z_start - 15*n*z_end + 15*n*z_start + 2*h*n4 - h*n5 + 10*n2*z_end - 10*n2*z_start)/(- n3*t2 + 4*n2*t2 - 5*n*t2 + 2*t2);
  poly_z_.c[Polynomial::E] =  0;
  poly_z_.c[Polynomial::F] =  z_start;
  poly_z_.duration = duration_;

}

void
EESwingMotion::SetDuration (double T)
{
  duration_ = T;
}

StateLin3d
EESwingMotion::GetState (double t_local) const
{
  StateLin1d z;
  poly_z_.GetPoint(/*t_start_z_ + */t_local, z);

  StateLin2d xy;
  poly_xy_.GetPoint(t_local, xy);

  StateLin3d ee;
  ee.SetDimension(xy.Get1d(X), X);
  ee.SetDimension(xy.Get1d(Y), Y);
  ee.SetDimension(z, Z);

  return ee;
}

} /* namespace opt */
} /* namespace xpp */
