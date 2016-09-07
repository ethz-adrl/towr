/**
 @file    linear_spline_equations_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 25, 2016
 @brief   Defines test for spline jacobians
 */

#include <xpp/zmp/linear_spline_equations.h>
#include <xpp/zmp/motion_factory.h>
#include <xpp/zmp/motion_structure.h>
#include <xpp/hyq/foothold.h>
#include <xpp/zmp/com_spline.h>

#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

using Coeff = Eigen::VectorXd;
using JacobianRow = ComMotion::JacobianRow;

TEST(LinearSplineEquations, AccelerationCostTest)
{
  // create the fixed motion structure
  MotionStructure motion_structure;
  motion_structure.Init({}, {hyq::LH, hyq::LF}, 0.7, 0.4, true, true);

  // create a spline made up of two fifth order polynomials
  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases());

  LinearSplineEquations eq(*com_motion);

  auto M = eq.MakeAcceleration(1.0, 1.0);

}

TEST(LinearSplineEquations, JunctionTestPosition)
{
  // create the fixed motion structure
  MotionStructure motion_structure;
  motion_structure.Init({}, {hyq::LH, hyq::LF}, 0.7, 0.4, true, true);

  // create a spline made up of two fifth order polynomials
  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases());
  auto com_spline = std::dynamic_pointer_cast<ComSpline>(com_motion);

  // duration of first polynomial
  double T = com_spline->GetPolynomial(0).GetDuration();

  // "approximating" the x-position as a linear function around zero coefficients
  com_spline->SetCoefficientsZero();
  // jacobian w.r.t spline coefficients at zero, time at T, plus the
  // position of the first spline at time T with zero coefficients
  JacobianRow jac_T = com_spline->GetJacobianWrtCoeffAtPolynomial(kPos, T, 0, X);
  double xT_at_0 = com_spline->GetCOGxyAtPolynomial(T,   0).GetByIndex(kPos, X);

  // jacobian w.r.t spline coefficients at zero, time at 0, plus the
  // position of the second spline at time 0 with zero coefficients
  JacobianRow jac_0 = com_spline->GetJacobianWrtCoeffAtPolynomial(kPos, 0.0, 1, X);
  double x0_at_0 = com_spline->GetCOGxyAtPolynomial(0.0, 1).GetByIndex(kPos, X);

  // now set coefficients so the junction at position is equal
  Coeff u = com_spline->GetCoeffients(); // zero
  double v0 = 1.0; // initial velocity
  u(com_spline->Index(0,X,E)) = v0;
  u(com_spline->Index(1,X,F)) = v0*T; // prediction where this spline will end up after T

  // double check if linear approximation yields same results
  double xT_at_u = (jac_T*u)[0]  + xT_at_0;
  double x0_at_u = (jac_0*u)[0]  + x0_at_0;

  EXPECT_DOUBLE_EQ(xT_at_u, x0_at_u);
}


} /* namespace zmp */
} /* namespace xpp */
