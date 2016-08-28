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

class LineEquationTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  Eigen::Vector2d A;
  Eigen::Vector2d B;
  Eigen::Vector2d C;
  Eigen::Vector2d D;
  LineEquation le;

};


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
