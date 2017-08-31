/**
 @file    height_map.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#include <xpp/height_map.h>

#include <cassert>
#include <cmath>

#include <xpp/cartesian_declarations.h>

namespace xpp {
namespace opt {


HeightMap::Ptr
HeightMap::MakeTerrain (ID type)
{
  switch (type) {
    case FlatID:    return std::make_shared<FlatGround>(); break;
    case StairsID:  return std::make_shared<Stairs>(); break;
    case GapID:     return std::make_shared<Gap>(); break;
    case SlopeID:   return std::make_shared<Slope>(); break;
    case ChimneyID: return std::make_shared<Chimney>(); break;
    default: assert(false); break;
  }
}

HeightMap::Vector3d
HeightMap::GetNormal (double x, double y, bool normalize) const
{
  Vector3d n;

  for (auto dim : {X,Y})
    n(dim) = -GetHeightFirstDerivativeWrt(dim, x, y);

  n(Z) = 1.0;

  if (normalize)
    n.normalize();

  return n;
}

HeightMap::Vector3d
HeightMap::GetNonNormalizedNormalDerivativeWrt (Coords3D deriv, double x, double y) const
{
  Vector3d n;

  for (auto dim : {X,Y})
    n(dim) = -GetHeightSecondDerivative(dim, deriv, x, y);

  n(Z) = 0.0;
  return n;
}

HeightMap::Vector3d
HeightMap::GetTangent1 (double x, double y) const
{
  Vector3d tx;
  tx(X) = 1.0;
  tx(Y) = 0.0;
  tx(Z) = GetHeightFirstDerivativeWrt(X, x,y);
  return tx;
}

HeightMap::Vector3d
HeightMap::GetTangent2 (double x, double y) const
{
  Vector3d ty;
  ty(X) = 0.0;
  ty(Y) = 1.0;
  ty(Z) = GetHeightFirstDerivativeWrt(Y,x,y);
  return ty;
}

HeightMap::Vector3d
HeightMap::GetNormalDerivativeWrt (Coords3D dim, double x, double y) const
{
  Vector3d n_non_normalized = GetNormal(x,y, false);
  Vector3d dn_normalized_wrt_n = GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(n_non_normalized, dim);
  Vector3d dn_wrt_dim = GetNonNormalizedNormalDerivativeWrt(dim, x, y);
  return dn_normalized_wrt_n.cwiseProduct(dn_wrt_dim);
}

//HeightMap::Vector3d
//HeightMap::GetNormalDerivativeWrtX (double x, double y) const
//{
//  Vector3d n = GetNormal(x,y);
//  Vector3d dn_normalized_wrt_n = GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(n, X);
//
//  Vector3d dn_wrt_x;
//  dn_wrt_x(X) = -GetHeightDerivWrtXX(x,y);
//  dn_wrt_x(Y) = -GetHeightDerivWrtYX(x,y);
//  dn_wrt_x(Z) = 0.0;
//
//  return dn_normalized_wrt_n.cwiseProduct(dn_wrt_x);
//}
//
//HeightMap::Vector3d
//HeightMap::GetNormalDerivativeWrtY (double x, double y) const
//{
//  Vector3d dndy;
//  dndy(X) = -GetHeightDerivWrtXY(x,y);
//  dndy(Y) = -GetHeightDerivWrtYY(x,y);
//  dndy(Z) = -0.0;
//  return dndy;
//}

HeightMap::Vector3d
HeightMap::GetTangent1DerivativeWrtX (double x, double y) const
{
  double dzdx =  GetHeightDerivWrtX(x,y);
  double dzdxx = GetHeightDerivWrtXX(x,y);

  Vector3d dtx_dx;
  dtx_dx(X) = GetOuterDerivativeWithOneInNumerator(dzdx);
  dtx_dx(Y) = 0.0;
  dtx_dx(Z) = GetOuterDerivativeWithValInNumerator(dzdx)*dzdxx;
  return dtx_dx;
}

HeightMap::Vector3d
HeightMap::GetTangent1DerivativeWrtY (double x, double y) const
{
  double dzdx  = GetHeightDerivWrtX(x,y);
  double dzdxy = GetHeightDerivWrtXX(x,y);

  Vector3d dtx_dy;
  dtx_dy(X) = GetOuterDerivativeWithOneInNumerator(dzdx);
  dtx_dy(Y) = 0.0;
  dtx_dy(Z) = GetOuterDerivativeWithValInNumerator(dzdx)*dzdxy;
  return dtx_dy;
}

HeightMap::Vector3d
HeightMap::GetTangent2DerivativeWrtX (double x, double y) const
{
  double dzdy  = GetHeightDerivWrtY(x,y);
  double dzdyx = GetHeightDerivWrtYX(x,y);

  Vector3d dty_dx;
  dty_dx(X) = 0.0;
  dty_dx(Y) = GetOuterDerivativeWithOneInNumerator(dzdy);
  dty_dx(Z) = GetOuterDerivativeWithValInNumerator(dzdy)*dzdyx;
  return dty_dx;
}

HeightMap::Vector3d
HeightMap::GetTangent2DerivativeWrtY (double x, double y) const
{
  double dzdy  = GetHeightDerivWrtY(x,y);
  double dzdyy = GetHeightDerivWrtYY(x,y);

  Vector3d dty_dy;
  dty_dy(X) = 0.0;
  dty_dy(Y) = GetOuterDerivativeWithOneInNumerator(dzdy);
  dty_dy(Z) = GetOuterDerivativeWithValInNumerator(dzdy)*dzdyy;
  return dty_dy;
}

double
HeightMap::GetOuterDerivativeWithValInNumerator (double x) const
{
  double x2 = x*x;
  return 1.0/std::pow(x2 + 1, 3./2.);
}

double
HeightMap::GetOuterDerivativeWithOneInNumerator (double x) const
{
  double x2 = x*x;
  return -x/std::pow(x2 + 1, 3./2.);
}

double
HeightMap::GetHeightFirstDerivativeWrt (Coords3D dim, double x, double y) const
{
  switch (dim) {
    case X: return GetHeightDerivWrtX(x,y);
    case Y: return GetHeightDerivWrtY(x,y);
    default: assert(false); // derivative dimension not implemented
  }
}

double
HeightMap::GetHeightSecondDerivative (Coords3D dim1, Coords3D dim2, double x, double y) const
{
  if (dim1 == X) {
    if (dim2 == X) return GetHeightDerivWrtXX(x,y);
    if (dim2 == Y) return GetHeightDerivWrtXY(x,y);
  } else {
    if (dim2 == X) return GetHeightDerivWrtYX(x,y);
    if (dim2 == Y) return GetHeightDerivWrtYY(x,y);
  }

  assert(false); // second derivative not specified.
}

HeightMap::Vector3d
HeightMap::GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex (
    const Vector3d& v, int idx) const
{
  return 1/v.squaredNorm()*(v.norm() * Vector3d::Unit(idx) - v(idx)*v.normalized());
}



// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>first_step_start_)
    h = height_first_step;

  if (x>first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}



// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (gap_start_ < x && x < gap_start_+gap_width_) {
    // try parabola for better gradients
    double dx = gap_width_/2;
    double x_center = gap_start_ + gap_width_/2;
    h = gap_depth_/(dx*dx) * std::pow(x - x_center,2) - gap_depth_;
  }

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ < x && x < gap_start_+gap_width_) {
    double dx = gap_width_/2;
    double x_center = gap_start_ + gap_width_/2;
    dhdx = 2*gap_depth_/(dx*dx) * (x - x_center);
  }

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ < x && x < gap_start_+gap_width_) {
    double dx = gap_width_/2;
    dzdxx = 2*gap_depth_/(dx*dx);
  }

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if (x > slope_start_)
    z = slope_*(x-slope_start_);

  // going back down
  if (x > x_down_start_) {
    z = height_center - slope_*(x-x_down_start_);
  }

  // back on flat ground
  if (x > x_flat_start_)
    z = 0.0;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x > slope_start_)
    dzdx = slope_;

  if (x > x_down_start_)
    dzdx = -slope_;

  if (x > x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}



double
Chimney::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x>x_start_ && x<x_start_+length_) {

    // slopes on the side
    if (y>y_start_) // left side
      z = slope_*(y-y_start_);
    else if (y<-y_start_) // right side
      z = -slope_*(y+y_start_);
    else
      z = z_depth_/(y_start_*y_start_) * y*y - z_depth_;
  }

  return z;
}


double
Chimney::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_ < x && x < x_start_+length_) {

    if (y>y_start_) // left side slope
      dzdy = slope_;
    else if (y<-y_start_) // right side slope
      dzdy = -slope_;
    else // center modeled as parabola
      dzdy = 2*z_depth_/(y_start_*y_start_) *y;
  }

  return dzdy;
}

double
Chimney::GetHeightDerivWrtYY (double x, double y) const
{
  double dzdyy = 0.0;

  if (x_start_ < x && x < x_start_+length_) {

    if (y>y_start_) // left side slope
      dzdyy = 0.0;
    else if (y<-y_start_) // right side slope
      dzdyy = 0.0;
    else // center modeled as parabola
      dzdyy = 2*z_depth_/(y_start_*y_start_);
  }

  return dzdyy;
}

} /* namespace opt */
} /* namespace xpp */


