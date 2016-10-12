/**
 @file    motion_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include "../include/xpp/opt/motion_factory.h"

#include "../include/xpp/opt/com_spline4.h" // the concrete classes
#include "../include/xpp/opt/com_spline6.h" // the concrete classes

namespace xpp {
namespace opt {

MotionFactory::MotionFactory ()
{
  // TODO Auto-generated constructor stub
}

MotionFactory::~MotionFactory ()
{
  // TODO Auto-generated destructor stub
}

MotionFactory::ComMotionPtrS
MotionFactory::CreateComMotion (const PhaseVec& phases)
{
  auto com_spline = std::make_shared<ComSpline6>();
  com_spline->Init(phases);
  return com_spline;
}

MotionFactory::ComMotionPtrS
MotionFactory::CreateComMotion (const PhaseVec& phases,
                                const Vector2d& start_cog_p,
                                const Vector2d& start_cog_v)
{
  auto com_spline = std::make_shared<ComSpline4>();
  com_spline->Init(phases);
  com_spline->SetStartPosVel(start_cog_p, start_cog_v);
  com_spline->SetEndAtStart();
  return com_spline;
}

} /* namespace zmp */
} /* namespace xpp */

