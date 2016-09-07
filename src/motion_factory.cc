/**
 @file    motion_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include <xpp/zmp/motion_factory.h>
#include <xpp/zmp/com_spline.h>
#include <xpp/zmp/com_spline6.h> // the concrete classes
#include <xpp/zmp/com_spline4.h> // the concrete classes

namespace xpp {
namespace zmp {

MotionFactory::MotionFactory ()
{
  // TODO Auto-generated constructor stub
}

MotionFactory::~MotionFactory ()
{
  // TODO Auto-generated destructor stub
}

MotionFactory::ComSplinePtr
MotionFactory::CreateComMotion (const PhaseVec& phases)
{
  auto com_spline = std::make_shared<ComSpline6>();
  com_spline->Init(phases);
  return com_spline;
}

MotionFactory::ComSplinePtr
MotionFactory::CreateComMotion (const PhaseVec& phases,
                                const Vector2d& start_cog_p,
                                const Vector2d& start_cog_v)
{
  auto com_spline = std::make_shared<ComSpline4>();
  com_spline->Init(start_cog_p, start_cog_v, phases);
  com_spline->SetEndAtStart();
  return com_spline;
}

MotionFactory::ComSplinePtr
MotionFactory::CreateComMotion (int step_count, const SplineTimes& times,
                                bool insert_initial_stance)
{
  auto com_spline = std::make_shared<ComSpline6>();
  com_spline->Init(step_count, times, insert_initial_stance);
  return com_spline;
}

MotionFactory::ComSplinePtr
MotionFactory::CreateComMotion (const Vector2d& start_cog_p,
                                const Vector2d& start_cog_v, int step_count,
                                const SplineTimes& times,
                                bool insert_initial_stance)
{
  auto com_spline = std::make_shared<ComSpline4>();
  com_spline->Init(start_cog_p, start_cog_v, step_count, times, insert_initial_stance);
  com_spline->SetEndAtStart();
  return com_spline;
}

} /* namespace zmp */
} /* namespace xpp */
