/**
 @file    height_map.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#include <xpp/height_map.h>

#include <cassert>
#include <cmath>
#include <vector>

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
HeightMap::GetNormalNotNormalized(double x, double y, const Derivatives& deriv) const
{
  Vector3d n;

  bool basis_requested = deriv.empty();

  for (auto dim : {X_,Y_}) {
    if (basis_requested)
      n(dim) = -GetDerivativeOfHeightWrt(dim, x, y);
    else
      n(dim) = -GetSecondDerivativeOfHeightWrt(dim, deriv.front(), x, y);
  }

  n(Z) = basis_requested? 1.0 : 0.0;

  return n;
}

HeightMap::Vector3d
HeightMap::GetTangent1NotNormalized (double x, double y, const Derivatives& deriv) const
{
  Vector3d tx;

  bool basis_requested = deriv.empty();

  tx(X) = basis_requested? 1.0 : 0.0;
  tx(Y) = 0.0;
  tx(Z) = basis_requested? GetDerivativeOfHeightWrt(X_, x,y) : GetSecondDerivativeOfHeightWrt(X_, deriv.front(), x, y);

  return tx;
}

HeightMap::Vector3d
HeightMap::GetTangent2NotNormalized (double x, double y, const Derivatives& deriv) const
{
  Vector3d ty;

  bool basis_requested = deriv.empty();

  ty(X) = 0.0;
  ty(Y) = basis_requested? 1.0 : 0.0;
  ty(Z) = basis_requested? GetDerivativeOfHeightWrt(Y_, x,y) : GetSecondDerivativeOfHeightWrt(Y_, deriv.front(), x, y);
  return ty;
}

HeightMap::Vector3d
HeightMap::GetBasisNotNormalized (BasisVector basis, double x, double y,
                                  const Derivatives& deriv) const
{
  switch (basis) {
    case Normal:   return GetNormalNotNormalized(x,y, deriv);
    case Tangent1: return GetTangent1NotNormalized(x,y, deriv);
    case Tangent2: return GetTangent2NotNormalized(x,y, deriv);
    default: assert(false); // basis does not exist
  }
}

HeightMap::Vector3d
HeightMap::GetNormalizedBasis (BasisVector basis, double x, double y) const
{
  return GetBasisNotNormalized(basis, x,y).normalized();
}

HeightMap::Vector3d
HeightMap::GetDerivativeOfNormalizedBasisWrt (BasisVector basis,
                                              Coords2D dim, double x, double y) const
{
  // inner derivative
  Vector3d dv_wrt_dim = GetBasisNotNormalized(basis, x, y, {dim});

  // outer derivative
  Vector3d v = GetBasisNotNormalized(basis, x,y, {});
  Vector3d dn_normalized_wrt_n = GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex(v, dim);
  return dn_normalized_wrt_n.cwiseProduct(dv_wrt_dim);
}

HeightMap::Vector3d
HeightMap::GetDerivativeOfNormalizedVectorWrtNonNormalizedIndex (const Vector3d& v, int idx) const
{
  // see notebook or
  // http://blog.mmacklin.com/2012/05/
  return 1/v.squaredNorm()*(v.norm() * Vector3d::Unit(idx) - v(idx)*v.normalized());
}

double
HeightMap::GetDerivativeOfHeightWrt (Coords2D dim, double x, double y) const
{
  switch (dim) {
    case X: return GetHeightDerivWrtX(x,y);
    case Y: return GetHeightDerivWrtY(x,y);
    default: assert(false); // derivative dimension not implemented
  }
}

double
HeightMap::GetSecondDerivativeOfHeightWrt (Coords2D dim1, Coords2D dim2, double x, double y) const
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

//  // modelled as parabola
//  if (gap_start_ < x && x < gap_start_+gap_width_) {
//    double dx = gap_width_/2;
//    double x_center = gap_start_ + gap_width_/2;
//    h = gap_depth_/(dx*dx) * std::pow(x - x_center,2) - gap_depth_;
//  }

  // modelled as straight slopes
  if (gap_start_ < x && x < gap_center_x_)
    h = -slope_*(x - gap_start_);

  if (gap_center_x_ < x && x < gap_start_+gap_width_)
    h = slope_*(x - gap_center_x_) - gap_depth_;


  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

//  //  modelled as parabola
//  if (gap_start_ < x && x < gap_start_+gap_width_) {
//    double dx = gap_width_/2;
//    double x_center = gap_start_ + gap_width_/2;
//    dhdx = 2*gap_depth_/(dx*dx) * (x - x_center);
//  }

  // modelled as straight slopes
  if (gap_start_ < x && x < gap_start_+gap_width_/2.)
    dhdx = -slope_;

  if (gap_start_+gap_width_/2. < x && x < gap_start_+gap_width_)
    dhdx = slope_;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

//  // modelled as parabola
//  if (gap_start_ < x && x < gap_start_+gap_width_) {
//    double dx = gap_width_/2;
//    dzdxx = 2*gap_depth_/(dx*dx);
//  }

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


