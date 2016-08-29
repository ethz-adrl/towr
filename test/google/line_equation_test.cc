/**
 @file    line_equation_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Brief description
 */

#include <xpp/utils/line_equation.h>
#include <gtest/gtest.h>

namespace xpp {
namespace utils {

using Point = LineEquation::Point;

class LineEquationTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  Point A;
  Point B;
  Point C;
  Point D;
  LineEquation le;

};

TEST_F(LineEquationTest, CoeffJacbianTest)
{
  A << 1.3, 0.5;
  B << -0.7, 2.9;

  // evaluate through numerical differences
  le.SetPoints(A,B);
  auto coeff_base = le.GetCoeff();

  double h = 1e-9;
  Point dx(h,  0.0);
  Point dy( 0.0, h);

  le.SetPoints(A+dx,B);
  auto coeff_0x = le.GetCoeff();

  le.SetPoints(A+dy,B);
  auto coeff_0y = le.GetCoeff();

  le.SetPoints(A,B+dx);
  auto coeff_1x = le.GetCoeff();

  le.SetPoints(A,B+dy);
  auto coeff_1y = le.GetCoeff();

  double grad_p0x = (coeff_0x.p - coeff_base.p)/h;
  double grad_p0y = (coeff_0y.p - coeff_base.p)/h;
  double grad_p1x = (coeff_1x.p - coeff_base.p)/h;
  double grad_p1y = (coeff_1y.p - coeff_base.p)/h;

  double grad_q0x = (coeff_0x.q - coeff_base.q)/h;
  double grad_q0y = (coeff_0y.q - coeff_base.q)/h;
  double grad_q1x = (coeff_1x.q - coeff_base.q)/h;
  double grad_q1y = (coeff_1y.q - coeff_base.q)/h;

  double grad_r0x = (coeff_0x.r - coeff_base.r)/h;
  double grad_r0y = (coeff_0y.r - coeff_base.r)/h;
  double grad_r1x = (coeff_1x.r - coeff_base.r)/h;
  double grad_r1y = (coeff_1y.r - coeff_base.r)/h;

  LineEquation::JacobianCoeff jac_numerical;
  jac_numerical.row(0) << grad_p0x, grad_p0y, grad_p1x, grad_p1y;
  jac_numerical.row(1) << grad_q0x, grad_q0y, grad_q1x, grad_q1y;
  jac_numerical.row(2) << grad_r0x, grad_r0y, grad_r1x, grad_r1y;

  le.SetPoints(A,B);
  auto jac_ana = le.GetJacobianLineCoeffWrtPoints();

  EXPECT_TRUE(jac_numerical.isApprox(jac_ana, 1e-5));
//  for (int row=0; row<3; row++)
//    for (int col=0; col<4; col++)
//      std::cout << jac_ana(row,col) << " vs " << jac_numerical(row,col) << std::endl;
}

TEST_F(LineEquationTest, DistanceJacbianTest)
{
  A << 1.3, 0.5;
  B << -0.7, 2.9;
  C << 0.2, 1.9;
  le.SetPoints(A,B);

  auto distance_base = le.GetDistanceFromLine(C);

  // perturb the points that define the line
  double h = 1e-9;
  Point dx(h,  0.0);
  Point dy( 0.0, h);

  le.SetPoints(A+dx,B);
  auto coeff_0x = le.GetDistanceFromLine(C);

  le.SetPoints(A+dy,B);
  auto coeff_0y = le.GetDistanceFromLine(C);

  le.SetPoints(A,B+dx);
  auto coeff_1x = le.GetDistanceFromLine(C);

  le.SetPoints(A,B+dy);
  auto coeff_1y = le.GetDistanceFromLine(C);

  double grad_d_0x = (coeff_0x - distance_base)/h;
  double grad_d_0y = (coeff_0y - distance_base)/h;
  double grad_d_1x = (coeff_1x - distance_base)/h;
  double grad_d_1y = (coeff_1y - distance_base)/h;

  LineEquation::JacobianRow jac_numerical;
  jac_numerical << grad_d_0x, grad_d_0y, grad_d_1x, grad_d_1y;

  le.SetPoints(A,B);
  auto jac_analytical = le.GetJacobianDistanceWrtPoints(C);

  EXPECT_TRUE(jac_numerical.isApprox(jac_analytical, 1e-5));
//  for (int col=0; col<4; col++)
//    std::cout << jac_analytical(0,col) << " vs " << jac_numerical(0,col) << std::endl;
}

TEST_F(LineEquationTest, LineCoefficients)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  LineCoeff2d line;
  // x-axis line (y=0)
  le.SetPoints(A,B);
  line = le.GetCoeff();
  EXPECT_DOUBLE_EQ(0, line.p);
  EXPECT_DOUBLE_EQ(1, line.q);
  EXPECT_DOUBLE_EQ(0, line.r);

  // shifted y-axis line (x=1)
  le.SetPoints(B,C);
  line = le.GetCoeff();
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 0, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);

  // diagonal line to top right
  le.SetPoints(A,C);
  line = le.GetCoeff();
  EXPECT_DOUBLE_EQ(-1/std::sqrt(2), line.p);
  EXPECT_DOUBLE_EQ( 1/std::sqrt(2), line.q);
  EXPECT_DOUBLE_EQ( 0, line.r);

  // diagonal line to top left
  le.SetPoints(B,D);
  line = le.GetCoeff();
  EXPECT_DOUBLE_EQ(-1/std::sqrt(2), line.p);
  EXPECT_DOUBLE_EQ(-1/std::sqrt(2), line.q);
  EXPECT_DOUBLE_EQ( 1/std::sqrt(2), line.r);
}

TEST_F(LineEquationTest, LineCoefficientsDistanceSign)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  // distance of point B and D to diagonal line A->C
  double distance_to_B, distance_to_D;
  le.SetPoints(A,C);
  distance_to_B = le.GetDistanceFromLine(B);
  distance_to_D = le.GetDistanceFromLine(D);

  EXPECT_TRUE(distance_to_B < 0.0); // because B right of AC
  EXPECT_TRUE(distance_to_D > 0.0); // because D left  of AC

  // reverse the direction of the line, now from C to A
  le.SetPoints(C,A);
  distance_to_B = le.GetDistanceFromLine(B);
  distance_to_D = le.GetDistanceFromLine(D);

  EXPECT_TRUE(distance_to_B > 0.0); // because B right of AC
  EXPECT_TRUE(distance_to_D < 0.0); // because D left  of AC
}

TEST_F(LineEquationTest, LineCoefficientsDistanceValue)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  // distance of point B and D to diagonal line A->C
  le.SetPoints(A,C);
  double distance_to_B = le.GetDistanceFromLine(B);
  double distance_to_D = le.GetDistanceFromLine(D);

  EXPECT_DOUBLE_EQ(hypot(0.5, 0.5), std::fabs(distance_to_B));
  EXPECT_DOUBLE_EQ(distance_to_B, -distance_to_D); // both on opposite sides of line
}


} /* namespace utils */
} /* namespace xpp */
