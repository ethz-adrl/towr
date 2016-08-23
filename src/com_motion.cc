/**
 @file    com_motion.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/zmp/com_motion.h>

namespace xpp {
namespace zmp {

ComMotion::ComMotion ()
{
  // TODO Auto-generated constructor stub
}

ComMotion::~ComMotion ()
{
  // TODO Auto-generated destructor stub
}

std::vector<double>
ComMotion::GetDiscretizedGlobalTimes() const
{
  static constexpr double dt = 0.1; //discretization time [seconds]: needed for creating support triangle inequality constraints
  static constexpr double eps = 1e-10; // maximum inaccuracy when adding double numbers

  std::vector<double> vec;
  double t = 0.0;
  while (t <= GetTotalTime()-dt+eps) { // still add the second to last time, even if rounding errors to to floating point arithmetics
    vec.push_back(t);
    t += dt;
  }

  vec.push_back(GetTotalTime());
  return vec;
}

int
ComMotion::GetTotalNodes() const
{
  return GetDiscretizedGlobalTimes().size();
}

} /* namespace zmp */
} /* namespace xpp */
