/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/opt/polynomial.h>

#include <gtest/gtest.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace utils {


// A start and an end position for the splines. checking only boundary conditions
class SplineTest : public ::testing::Test {
public:
  using Point = StateLin1d;

protected:
  static void SetUpTestCase() // this is now only done one for all test in this test case
  {
    start.p   =  5.0;
    start.v  = -1.0;
    start.a =  3.0;

    end.p   =  4.0;
    end.v  =  2.0;
    end.a =  2.2;

    T = 3.2;
  }

  static Point start;
  static Point end;
  static double T;
};

SplineTest::Point SplineTest::start = Point();
SplineTest::Point SplineTest::end = Point();
double SplineTest::T = 0.0;


TEST_F(SplineTest, LinearPolynomial)
{
  LinearPolynomial s;
  s.SetBoundary(T, start, end);

  Point p0, p1;
  s.GetPoint(0.0, p0);
  s.GetPoint(T, p1);

  // pos
  EXPECT_FLOAT_EQ(start.p, p0.p);
  EXPECT_FLOAT_EQ(end.p, p1.p);
  // vel
  EXPECT_FLOAT_EQ(p0.v, p1.v);
  // acc
  EXPECT_FLOAT_EQ(0.0, p0.a);
  EXPECT_FLOAT_EQ(0.0, p1.a);
}


TEST_F(SplineTest, CubicPolynomial)
{
  CubicPolynomial s;
  s.SetBoundary(T, start, end);

  Point p0, p1;
  s.GetPoint(0.0, p0);
  s.GetPoint(T, p1);

  // pos
  EXPECT_FLOAT_EQ(start.p, p0.p);
  EXPECT_FLOAT_EQ(end.p, p1.p);
  // vel
  EXPECT_FLOAT_EQ(start.v, p0.v);
  EXPECT_FLOAT_EQ(end.v, p1.v);
}


TEST_F(SplineTest, FifthOrderSpliner)
{
  QuinticPolynomial s;
  s.SetBoundary(T, start, end);

  Point p0, p1;
  s.GetPoint(0.0, p0);
  s.GetPoint(T, p1);

  // pos
  EXPECT_FLOAT_EQ(start.p, p0.p);
  EXPECT_FLOAT_EQ(end.p, p1.p);
  // vel
  EXPECT_FLOAT_EQ(start.v, p0.v);
  EXPECT_FLOAT_EQ(end.v, p1.v);
  // acc
  EXPECT_FLOAT_EQ(start.a, p0.a);
  EXPECT_FLOAT_EQ(end.a, p1.a);
}


TEST_F(SplineTest, FifthOrderSplinerSameStartGoal)
{
  QuinticPolynomial s;

  Point p;
  p.p   = 2.0;
  p.v  = 2.0;
  p.a = 2.0;

  Point pg;
  pg.p   = 2.0;
  pg.v  = 2.0;
  pg.a = 2.0;

  double t = 0.001;
  s.SetBoundary(t, p, pg);

  Point p0,p2;
  s.GetPoint(0.0,   p0);
  s.GetPoint(t,     p2);

  // pos
  EXPECT_FLOAT_EQ(2.0, p0.p);
  EXPECT_FLOAT_EQ(2.0, p2.p);
  // vel
  EXPECT_FLOAT_EQ(2.0, p0.v);
  EXPECT_FLOAT_EQ(2.0, p2.v);
  // acc
  EXPECT_FLOAT_EQ(2.0, p0.a);
  EXPECT_FLOAT_EQ(2.0, p2.a);
}


} // namespace utils
} // namespace xpp
