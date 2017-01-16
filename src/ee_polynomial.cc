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
}

EEPolynomial::~EEPolynomial ()
{
  // TODO Auto-generated destructor stub
}

void
EEPolynomial::SetParams (const XYState& start, const XYState& end,
                                double z_max, double t_swing)
{
  // Setting 6 constraints:
  // initial pos/vel is zero
  // pos,vel at halftime is z_max and zero
  // pos,vel at t_step is zero
  // see matlab script "matlab_z_height.m" for generation of these values
  poly_z_.c[utils::Polynomial::A] = 0;
  poly_z_.c[utils::Polynomial::B] =  16*z_max /std::pow(t_swing,4);
  poly_z_.c[utils::Polynomial::C] = -32*z_max /std::pow(t_swing,3);
  poly_z_.c[utils::Polynomial::D] =  16*z_max /std::pow(t_swing,2);
  poly_z_.c[utils::Polynomial::E] = 0;
  poly_z_.c[utils::Polynomial::F] = 0;
  poly_z_.duration = t_swing;

  poly_xy_.SetBoundary(t_swing, start, end);
}

EEPolynomial::XYZState
EEPolynomial::GetState (double t_local) const
{
  ZState z;
  poly_z_.GetPoint(t_local, z);

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
