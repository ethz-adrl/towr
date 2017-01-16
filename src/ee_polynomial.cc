/**
 @file    ee_height_z_polynomial.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 16, 2017
 @brief   Brief description
 */

#include "../include/xpp/opt/ee_polynomial.h"

namespace xpp {
namespace opt {

EEPolynomial::EEPolynomial ()
{
  duration_ = 0.0;
}

EEPolynomial::~EEPolynomial ()
{
  // TODO Auto-generated destructor stub
}

void
EEPolynomial::SetZParams (double p, double z_max)
{
  double t_z_total = duration_/(1-p);
  // Setting 6 constraints:
  // initial pos/vel is zero
  // pos,vel at halftime is z_max and zero
  // pos,vel at t_step is zero
  // see matlab script "matlab_z_height.m" for generation of these values
  poly_z_.c[utils::Polynomial::A] = 0;
  poly_z_.c[utils::Polynomial::B] =  16*z_max /std::pow(t_z_total,4);
  poly_z_.c[utils::Polynomial::C] = -32*z_max /std::pow(t_z_total,3);
  poly_z_.c[utils::Polynomial::D] =  16*z_max /std::pow(t_z_total,2);
  poly_z_.c[utils::Polynomial::E] = 0;
  poly_z_.c[utils::Polynomial::F] = 0;
  poly_z_.duration = t_z_total;

  t_start_z_ = t_z_total - duration_;
}

void
EEPolynomial::SetXYParams (const XYState& start, const XYState& end)
{
  poly_xy_.SetBoundary(duration_, start, end);
}

void
EEPolynomial::SetDuration (double T)
{
  duration_ = T;
}

EEPolynomial::XYZState
EEPolynomial::GetState (double t_local) const
{
  ZState z;
  poly_z_.GetPoint(t_start_z_ + t_local, z);

  XYState xy;
  poly_xy_.GetPoint(t_local, xy);

  XYZState ee;
  ee.SetDimension(xy.Get1d(utils::X), utils::X);
  ee.SetDimension(xy.Get1d(utils::Y), utils::Y);
  ee.SetDimension(z, utils::Z);

  return ee;
}

} /* namespace opt */
} /* namespace xpp */
