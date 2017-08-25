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
    case FlatID:   return std::make_shared<FlatGround>(); break;
    case StairsID: return std::make_shared<Stairs>(); break;
    case GapID:    return std::make_shared<Gap>(); break;
    case SlopeID:  return std::make_shared<Slope>(); break;
    default: assert(false); break;
  }
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

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x > slope_start_)
    dzdx = slope_;

  return dzdx;
}

double
Slope::GetHeightDerivWrtY (double x, double y) const
{
  return 0.0;
}

} /* namespace opt */
} /* namespace xpp */

