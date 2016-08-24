/**
@file    com_spline6.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 21, 2015
@brief   Defines ComSpline6, which realizes a ComSpline
 */

#include <xpp/zmp/com_spline6.h>

namespace xpp {
namespace zmp {

ComSpline6::ComSpline6 ()
{
  // TODO Auto-generated constructor stub
}

ComSpline6::~ComSpline6 ()
{
  // TODO Auto-generated destructor stub
}

void
ComSpline6::Init (int step_count, const SplineTimes& times,
                  bool insert_initial_stance)
{
  ComSpline::Init(step_count, times, insert_initial_stance);

  // initialize all coefficients to zero
  Eigen::VectorXd abcd(GetTotalFreeCoeff());
  abcd.setZero();
  SetCoefficients(abcd);
}

} // namespace zmp
} // namespace xpp


