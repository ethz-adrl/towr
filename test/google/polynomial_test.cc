/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/utils/polynomial.h>

#include <gtest/gtest.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace utils {


// A start and an end position for the splines. checking only boundary conditions
class SplineTest : public ::testing::Test {
public:
  typedef Polynomial::Point1d Point;

protected:
  static void SetUpTestCase() // this is now only done one for all test in this test case
  {
    start.x   =  5.0;
    start.xd  = -1.0;
    start.xdd =  3.0;

    end.x   =  4.0;
    end.xd  =  2.0;
    end.xdd =  2.2;

    T = 3.2;
  }

  static Polynomial::Point1d start;
  static Polynomial::Point1d end;
  static double T;
};

Polynomial::Point1d SplineTest::start = Point();
Polynomial::Point1d SplineTest::end = Point();
double SplineTest::T = 0.0;


TEST_F(SplineTest, LinearPolynomial)
{
  LinearPolynomial s;
  s.SetBoundary(T, start, end);

  Polynomial::Point1d p0, p1;
  s.GetPoint(0.0, p0);
  s.GetPoint(T, p1);

  // pos
  EXPECT_FLOAT_EQ(start.x, p0.x);
  EXPECT_FLOAT_EQ(end.x, p1.x);
  // vel
  EXPECT_FLOAT_EQ(p0.xd, p1.xd);
  // acc
  EXPECT_FLOAT_EQ(0.0, p0.xdd);
  EXPECT_FLOAT_EQ(0.0, p1.xdd);
}


TEST_F(SplineTest, CubicPolynomial)
{
  CubicPolynomial s;
  s.SetBoundary(T, start, end);

  Polynomial::Point1d p0, p1;
  s.GetPoint(0.0, p0);
  s.GetPoint(T, p1);

  // pos
  EXPECT_FLOAT_EQ(start.x, p0.x);
  EXPECT_FLOAT_EQ(end.x, p1.x);
  // vel
  EXPECT_FLOAT_EQ(start.xd, p0.xd);
  EXPECT_FLOAT_EQ(end.xd, p1.xd);
}


TEST_F(SplineTest, FifthOrderSpliner)
{
  QuinticPolynomial s;
  s.SetBoundary(T, start, end);

  Polynomial::Point1d p0, p1;
  s.GetPoint(0.0, p0);
  s.GetPoint(T, p1);

  // pos
  EXPECT_FLOAT_EQ(start.x, p0.x);
  EXPECT_FLOAT_EQ(end.x, p1.x);
  // vel
  EXPECT_FLOAT_EQ(start.xd, p0.xd);
  EXPECT_FLOAT_EQ(end.xd, p1.xd);
  // acc
  EXPECT_FLOAT_EQ(start.xdd, p0.xdd);
  EXPECT_FLOAT_EQ(end.xdd, p1.xdd);
}


TEST_F(SplineTest, FifthOrderSplinerSameStartGoal)
{
  QuinticPolynomial s;

  Point p;
  p.x   = 2.0;
  p.xd  = 2.0;
  p.xdd = 2.0;

  Point pg;
  pg.x   = 2.0;
  pg.xd  = 2.0;
  pg.xdd = 2.0;

  double t = 0.001;
  s.SetBoundary(t, p, pg);

  Polynomial::Point1d p0,p2;
  s.GetPoint(0.0,   p0);
  s.GetPoint(t,     p2);

  // pos
  EXPECT_FLOAT_EQ(2.0, p0.x);
  EXPECT_FLOAT_EQ(2.0, p2.x);
  // vel
  EXPECT_FLOAT_EQ(2.0, p0.xd);
  EXPECT_FLOAT_EQ(2.0, p2.xd);
  // acc
  EXPECT_FLOAT_EQ(2.0, p0.xdd);
  EXPECT_FLOAT_EQ(2.0, p2.xdd);
}


} // namespace utils
} // namespace xpp
