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
}

EESwingMotion::~EESwingMotion ()
{
}

void
EESwingMotion::Init (double T, double h, const Vector3d& start,
                     const Vector3d& end)
{
  T_ = T;
  h_ = h;
  SetContacts(start, end);
}

double
EESwingMotion::GetDuration () const
{
  return T_;
}

void
EESwingMotion::SetContacts (const Vector3d& start_pos, const Vector3d& end_pos)
{
  StateLin3d start_state(start_pos);
  StateLin3d end_state(end_pos);
  poly_xy_.SetBoundary(T_, start_state.Get2D(), end_state.Get2D());

  double n2 = std::pow(n_,2);
  double n3 = std::pow(n_,3);
  double n4 = std::pow(n_,4);
  double n5 = std::pow(n_,5);

  double t2 = std::pow(T_,2);
  double t3 = std::pow(T_,3);
  double t4 = std::pow(T_,4);
  double t5 = std::pow(T_,5);

  double z_start = start_pos.z();
  double z_end = end_pos.z();
  // see matlab script "swingleg_z_height.m" for generation of these values
  poly_z_.c[Polynomial::A] = -(2*(2*n2*z_end - 3*n3*z_end - 2*n2*z_start + 3*n3*z_start))/(t5*(n_ - 2)*(n2 - 2*n_ + 1));
  poly_z_.c[Polynomial::B] = -(2*h_*n4 - h_*n5 - 10*n2*z_end + 15*n3*z_end + 10*n2*z_start - 15*n3*z_start)/(t4*(n_ - 2)*(n2 - 2*n_ + 1));
  poly_z_.c[Polynomial::C] =  (2*(2*z_end - 2*z_start - 5*n_*z_end + 5*n_*z_start + 2*h_*n4 - h_*n5 + 5*n3*z_end - 5*n3*z_start))/((n_ - 2)*(n2*t3 - 2*n_*t3 + t3));
  poly_z_.c[Polynomial::D] =  (6*z_end - 6*z_start - 15*n_*z_end + 15*n_*z_start + 2*h_*n4 - h_*n5 + 10*n2*z_end - 10*n2*z_start)/(- n3*t2 + 4*n2*t2 - 5*n_*t2 + 2*t2);
  poly_z_.c[Polynomial::E] =  0;
  poly_z_.c[Polynomial::F] =  z_start;
  poly_z_.duration = T_;
}

StateLin3d
EESwingMotion::GetState (double t_local) const
{
  StateLin1d z;
  poly_z_.GetPoint(t_local, z);

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
