/**
 @file    line_equation_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 28, 2016
 @brief   Brief description
 */

#include <xpp/zmp/line_equation.h>
#include <gtest/gtest.h>

namespace xpp {
namespace utils {

class LineEquationTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  Eigen::Vector2d A;
  Eigen::Vector2d B;
  Eigen::Vector2d C;
  Eigen::Vector2d D;

};

TEST_F(LineEquationTest, LineCoeffNormalize)
{
  A << 0, 1;
  B << 2, 1;
  C << 0, 3;

  LineCoeff2d line_norm, line;
  line      = LineEquation::LineCoeff(A,B, false);
  line_norm = LineEquation::LineCoeff(A,B, true);

  // test distance to point
  double distance_to_C = line.p*C.x() + line.q*C.y() + line.r;
  double distance_to_C_norm = line_norm.p*C.x() + line_norm.q*C.y() + line_norm.r;

  EXPECT_NE(2.0, distance_to_C); // because line is not normalized
  EXPECT_DOUBLE_EQ(2.0, distance_to_C_norm);
}

TEST_F(LineEquationTest, LineCoefficients)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  LineCoeff2d line;
  // x-axis line (y=0)
  line = LineEquation::LineCoeff(A,B);
  EXPECT_DOUBLE_EQ(0, line.p);
  EXPECT_DOUBLE_EQ(1, line.q);
  EXPECT_DOUBLE_EQ(0, line.r);

  // shifted y-axis line (x=1)
  line = LineEquation::LineCoeff(B,C);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 0, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);

  // diagonal line to top right not normalized
  line = LineEquation::LineCoeff(A,C,false);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 1, line.q);
  EXPECT_DOUBLE_EQ( 0, line.r);

  // diagonal line to top left not normalized
  line = LineEquation::LineCoeff(B,D,false);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ(-1, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);
}

TEST_F(LineEquationTest, LineCoefficientsDistanceSign)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  // distance of point B and D to diagonal line A->C
  double distance_to_B, distance_to_D;
  xpp::utils::LineCoeff2d AC = LineEquation::LineCoeff(A,C);
  distance_to_B = AC.p*B.x() + AC.q*B.y() + AC.r;
  distance_to_D = AC.p*D.x() + AC.q*D.y() + AC.r;

  EXPECT_TRUE(distance_to_B < 0.0); // because B right of AC
  EXPECT_TRUE(distance_to_D > 0.0); // because D left  of AC


  // reverse the direction of the line, now from C to A
  xpp::utils::LineCoeff2d CA = LineEquation::LineCoeff(C,A);
  distance_to_B = CA.p*B.x() + CA.q*B.y() + CA.r;
  distance_to_D = CA.p*D.x() + CA.q*D.y() + CA.r;

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
  xpp::utils::LineCoeff2d AC = LineEquation::LineCoeff(A,C);
  double distance_to_B = AC.p*B.x() + AC.q*B.y() + AC.r;
  double distance_to_D = AC.p*D.x() + AC.q*D.y() + AC.r;

  EXPECT_DOUBLE_EQ(hypot(0.5, 0.5), std::fabs(distance_to_B));
  EXPECT_DOUBLE_EQ(distance_to_B, -distance_to_D); // both on opposite sides of line
}


} /* namespace utils */
} /* namespace xpp */
