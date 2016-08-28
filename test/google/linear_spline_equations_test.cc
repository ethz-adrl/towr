/**
 @file    linear_spline_equations_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Defines test for spline jacobians
 */

#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/motion_factory.h>
#include <xpp/zmp/com_spline.h>
#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

typedef Eigen::VectorXd Coeff;

TEST(LinearSplineEquations, JunctionTestPosition)
{
  // create a spline made up of two fifth order polynomials
  auto com_spline = MotionFactory::CreateComMotion(2, SplineTimes(0.7,0.4), false);

  // duration of first polynomial
  double T = com_spline->GetPolynomial(0).GetDuration();

  // "approximating" the x-position as a linear function around zero coefficients
  com_spline->SetCoefficientsZero();
  // jacobian w.r.t spline coefficients at zero, time at T, plus the
  // position of the first spline at time T with zero coefficients
  auto jac_T = com_spline->GetJacobianWrtCoeffAtPolynomial(kPos, T, 0, X);
  double xT_at_0 = com_spline->GetCOGxyAtPolynomial(T,   0).GetByIndex(kPos, X);

  // jacobian w.r.t spline coefficients at zero, time at 0, plus the
  // position of the second spline at time 0 with zero coefficients
  auto jac_0 = com_spline->GetJacobianWrtCoeffAtPolynomial(kPos, 0.0, 1, X);
  double x0_at_0 = com_spline->GetCOGxyAtPolynomial(0.0, 1).GetByIndex(kPos, X);

  // now set coefficients so the junction at position is equal
  Coeff u = com_spline->GetCoeffients(); // zero
  double v0 = 1.0; // initial velocity
  u(com_spline->Index(0,X,E)) = v0;
  u(com_spline->Index(1,X,F)) = v0*T; // prediction where this spline will end up after T

  // double check if linear approximation yields same results
  double xT_at_u = jac_T*u + xT_at_0;
  double x0_at_u = jac_0*u + x0_at_0;

  EXPECT_DOUBLE_EQ(xT_at_u, x0_at_u);
}

} /* namespace zmp */
} /* namespace xpp */
