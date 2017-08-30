/**
 @file    height_map.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#include <xpp/height_map.h>

#include <cassert>
#include <cmath>

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
  // calculate tangent vectors from gradients
  double dzdx = GetHeightDerivWrtX(x,y);
  double dzdy = GetHeightDerivWrtY(x,y);
  Vector3d tangent_x(1,0,dzdx);
  Vector3d tangent_y(0,1,dzdy);

  return tangent_x.cross(tangent_y);
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

double
Stairs::GetHeightDerivWrtX (double x, double y) const
{
  return 0.0;
}

double
Stairs::GetHeightDerivWrtY (double x, double y) const
{
  return 0.0;
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
Gap::GetHeightDerivWrtY (double x, double y) const
{
  return 0.0;
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
Slope::GetHeightDerivWrtY (double x, double y) const
{
  return 0.0;
}

double
Chimney::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x>x_start_ && x<x_start_+length_) {
    // gap
//    z = -z_depth_; // make parabola for better derivatives

    z = z_depth_/(y_start_*y_start_) * y*y - z_depth_;

    // slopes on the side
    if (y>y_start_) // left side
      z = slope_*(y-y_start_);
    if (y<-y_start_) // right side
      z = -slope_*(y+y_start_);
  }


  return z;
}

double
Chimney::GetHeightDerivWrtX (double x, double y) const
{
  return 0.0;
}

double
Chimney::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x>x_start_ && x<x_start_+length_) {

    // derivative of parabola
    dzdy = 2*z_depth_/(y_start_*y_start_) *y;


    // slopes on the side
    if (y>y_start_) // left side
      dzdy = slope_;
    if (y<-y_start_) // right side
      dzdy = -slope_;
  }

  return dzdy;
}

} /* namespace opt */
} /* namespace xpp */


