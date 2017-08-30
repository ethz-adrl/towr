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
HeightMap::GetNormal (double x, double y) const
{
  double dzdx = GetHeightDerivWrtX(x,y);
  double dzdy = GetHeightDerivWrtY(x,y);

//  // calculate tangent vectors from gradients
//  Vector3d tangent_x(1,0,dzdx);
//  Vector3d tangent_y(0,1,dzdy);
//  Vector3d normal = tangent_y.cross(tangent_x);

  return Vector3d(-dzdx, -dzdy, 1.0); // could also multiply by -1
}

HeightMap::Vector3d
HeightMap::GetNormalDerivativeWrtX (double x, double y) const
{
  Vector3d dndx = Vector3d::Zero();
  dndx(X) = -GetHeightDerivWrtXX(x,y);
  dndx(Y) = -GetHeightDerivWrtYX(x,y);
  return dndx;
}

HeightMap::Vector3d
HeightMap::GetNormalDerivativeWrtY (double x, double y) const
{
  Vector3d dndy = Vector3d::Zero();
  dndy(X) = -GetHeightDerivWrtXY(x,y);
  dndy(Y) = -GetHeightDerivWrtYY(x,y);
  return dndy;
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


