/**
 @file    motion_factory.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_factory.h>
#include <xpp/opt/com_spline6.h>

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
MotionFactory::CreateComMotion (double t_total,
                                int polynomials_per_second,
                                double height)
{
  auto com_spline = std::make_shared<ComSpline6>();
  com_spline->SetConstantHeight(height);

  com_spline->Init(t_total, polynomials_per_second);
  return com_spline;
}

} /* namespace opt */
} /* namespace xpp */

