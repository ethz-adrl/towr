/**
 @file    height_map.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2017
 @brief   Brief description
 */

#include <xpp/height_map.h>

#include <cassert>

namespace xpp {
namespace opt {

HeightMap::HeightMap ()
{
  // TODO Auto-generated constructor stub

}

HeightMap::~HeightMap ()
{
  // TODO Auto-generated destructor stub
}

HeightMap::Ptr
HeightMap::MakeTerrain (ID type)
{
  switch (type) {
    case Flat:   return std::make_shared<HeightMapFlat>(); break;
    case Stairs: return std::make_shared<HeightMapStairs>(); break;
    default: assert(false); break;
  }
}


HeightMapStairs::HeightMapStairs()
{
  slope_ = 0.7;
  slope_start_ = 0.4;
}

double
HeightMapStairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>slope_start_) {
//    h = slope_*(x-slope_start_);
    h = 0.3;
  }

  if (x>slope_start_+0.4) {
//    h = slope_*(x-slope_start_);
    h = 0.6;
  }

  return h;
}

double
HeightMapStairs::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

//  if (x>slope_start_)
//    dhdx = slope_;

  return dhdx;
}

double
HeightMapStairs::GetHeightDerivWrtY (double x, double y) const
{
  return 0.0;
}

HeightMapStairs::~HeightMapStairs()
{
}

} /* namespace opt */
} /* namespace xpp */


