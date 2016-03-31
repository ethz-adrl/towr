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
class PointSortingTest : public ::testing::Test {
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

    q3.x() = 0;
    q3.y() = 1;
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


TEST_F(PointSortingTest, SortByXThenY)
{
  EXPECT_TRUE(Point2dManip::P1LeftofP2(p0,p1));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(p1,p0));

  EXPECT_TRUE(Point2dManip::P1LeftofP2(p1,p2));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(p2,p1));

  EXPECT_TRUE(Point2dManip::P1LeftofP2(p0,p2));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(p2,p0));
}

TEST_F(PointSortingTest, 3PointsSorted)
{
  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = p0;
  points.at(1) = p1;
  points.at(2) = p2;

  std::vector<size_t> idx = Point2dManip::CounterClockwiseSort(points);

  EXPECT_EQ(idx.at(0), 0);
  EXPECT_EQ(idx.at(1), 1);
  EXPECT_EQ(idx.at(2), 2);
}

TEST_F(PointSortingTest, 3PointsUnsorted1)
{
  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = p1;
  points.at(1) = p2;
  points.at(2) = p0;

  std::vector<size_t> idx = Point2dManip::CounterClockwiseSort(points);

  // original index of the point, so {2,0,1}
  EXPECT_EQ(idx.at(0), 2);
  EXPECT_EQ(idx.at(1), 0);
  EXPECT_EQ(idx.at(2), 1);
}

TEST_F(PointSortingTest, 3PointsUnsorted2)
{
  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = p2;
  points.at(1) = p0;
  points.at(2) = p1;

  std::vector<size_t> idx = Point2dManip::CounterClockwiseSort(points);

  EXPECT_EQ(idx.at(0), 1);
  EXPECT_EQ(idx.at(1), 2);
  EXPECT_EQ(idx.at(2), 0);
}

// Testing with four points as used in four leg support polygons
TEST_F(PointSortingTest, 4PointsSorted)
{
  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = p0;
  points.at(1) = p1;
  points.at(2) = p2;
  points.at(3) = p3;

  std::vector<size_t> idx = Point2dManip::CounterClockwiseSort(points);

  EXPECT_EQ(idx.at(0), 0);
  EXPECT_EQ(idx.at(1), 1);
  EXPECT_EQ(idx.at(2), 2);
  EXPECT_EQ(idx.at(3), 3);
}

TEST_F(PointSortingTest, 4PointsUnsorted)
{
  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = p1;
  points.at(1) = p3;
  points.at(2) = p2;
  points.at(3) = p0;

  std::vector<size_t> idx = Point2dManip::CounterClockwiseSort(points);

  EXPECT_EQ(idx.at(0), 3);
  EXPECT_EQ(idx.at(1), 0);
  EXPECT_EQ(idx.at(2), 2);
  EXPECT_EQ(idx.at(3), 1);
}


} // namespace utils
} // namespace xpp
