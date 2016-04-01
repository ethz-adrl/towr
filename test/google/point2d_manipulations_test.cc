/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/utils/point2d_manipulations.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace utils {

//typedef Spliner::Point Point;
//
// A start and an end position for the splines. checking only boundary conditions
class Point2dManipulationsTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    p0.x() = 0;
    p0.y() = 0;

    p1.x() = 1;
    p1.y() = 0;

    p2.x() = 1;
    p2.y() = 1;

    p3.x() = 0;
    p3.y() = 1;


    q0.x() = -1;
    q0.y() = -1;

    q1.x() =  1;
    q1.y() = -2;

    q2.x() = -0.1;
    q2.y() =  0.1;

    // this point lies inside the convec hull of the others
    q3.x() = 0;
    q3.y() = -0.5;
  }

  // points arranged in positive square starting at origin
  Eigen::Vector2d p0;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
  Eigen::Vector2d p3;

  // points that don't create a convex hull, but one can be eliminated
  Eigen::Vector2d q0;
  Eigen::Vector2d q1;
  Eigen::Vector2d q2;
  Eigen::Vector2d q3;
};

TEST_F(Point2dManipulationsTest, LineCoefficients)
{
  LineCoeff2d line;
  // x-axis line (y=0)
  line = Point2dManip::LineCoeff(p0,p1);
  EXPECT_DOUBLE_EQ(0, line.p);
  EXPECT_DOUBLE_EQ(1, line.q);
  EXPECT_DOUBLE_EQ(0, line.r);

  // shifted y-axis line (x=1)
  line = Point2dManip::LineCoeff(p1,p2);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 0, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);

  // diagonal line to top right not normalized
  line = Point2dManip::LineCoeff(p0,p2,false);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 1, line.q);
  EXPECT_DOUBLE_EQ( 0, line.r);

  // diagonal line to top left not normalized
  line = Point2dManip::LineCoeff(p1,p3,false);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ(-1, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);
}

TEST_F(Point2dManipulationsTest, SortByXThenY)
{
  EXPECT_TRUE(Point2dManip::P1LeftofP2(p0,p1));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(p1,p0));

  EXPECT_TRUE(Point2dManip::P1LeftofP2(p1,p2));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(p2,p1));

  EXPECT_TRUE(Point2dManip::P1LeftofP2(p0,p2));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(p2,p0));
}

TEST_F(Point2dManipulationsTest, 3PointsSorted)
{
  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = p0;
  points.at(1) = p1;
  points.at(2) = p2;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);

  EXPECT_EQ(0, idx.at(0));
  EXPECT_EQ(1, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
}

TEST_F(Point2dManipulationsTest, 3PointsUnsorted1)
{
  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = p1;
  points.at(1) = p2;
  points.at(2) = p0;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(3, idx.size());

  // original index of the point, so {2,0,1}
  EXPECT_EQ(2, idx.at(0));
  EXPECT_EQ(0, idx.at(1));
  EXPECT_EQ(1, idx.at(2));
}

TEST_F(Point2dManipulationsTest, 3PointsUnsorted2)
{
  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = p2;
  points.at(1) = p0;
  points.at(2) = p1;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(3, idx.size());

  EXPECT_EQ(1, idx.at(0));
  EXPECT_EQ(2, idx.at(1));
  EXPECT_EQ(0, idx.at(2));
}

// Testing with four points as used in four leg support polygons
TEST_F(Point2dManipulationsTest, 4PointsConvexSorted)
{
  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = p0;
  points.at(1) = p1;
  points.at(2) = p2;
  points.at(3) = p3;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(4, idx.size());

  EXPECT_EQ(0, idx.at(0));
  EXPECT_EQ(1, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
  EXPECT_EQ(3, idx.at(3));
}

TEST_F(Point2dManipulationsTest, 4PointsConvexUnsorted)
{
  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = p1;
  points.at(1) = p3;
  points.at(2) = p2;
  points.at(3) = p0;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(4, idx.size());

  EXPECT_EQ(3, idx.at(0));
  EXPECT_EQ(0, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
  EXPECT_EQ(1, idx.at(3));
}

TEST_F(Point2dManipulationsTest, 4PointsNonConvex)
{
  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = q0;
  points.at(1) = q1;
  points.at(2) = q2;
  points.at(3) = q3;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(3, idx.size());

  EXPECT_EQ(0, idx.at(0));
  EXPECT_EQ(1, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
}

TEST_F(Point2dManipulationsTest, 4PointsEqual)
{
  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = q0;
  points.at(1) = q0;
  points.at(2) = q0;
  points.at(3) = q0;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(1, idx.size());

  EXPECT_EQ(0, idx.at(0));
}


} // namespace utils
} // namespace xpp
