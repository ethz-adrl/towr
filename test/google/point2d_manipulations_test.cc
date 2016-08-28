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


class Point2dManipulationsTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  Eigen::Vector2d A;
  Eigen::Vector2d B;
  Eigen::Vector2d C;
  Eigen::Vector2d D;

};

TEST_F(Point2dManipulationsTest, SortByXThenY)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  EXPECT_TRUE(Point2dManip::P1LeftofP2(A,B));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(B,A));

  EXPECT_TRUE(Point2dManip::P1LeftofP2(B,C));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(C,B));

  EXPECT_TRUE(Point2dManip::P1LeftofP2(A,C));
  EXPECT_FALSE(Point2dManip::P1LeftofP2(C,A));
}

TEST_F(Point2dManipulationsTest, 3PointsSorted)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;

  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = A;
  points.at(1) = B;
  points.at(2) = C;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);

  EXPECT_EQ(0, idx.at(0));
  EXPECT_EQ(1, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
}

TEST_F(Point2dManipulationsTest, 3PointsUnsorted1)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;

  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = B;
  points.at(1) = C;
  points.at(2) = A;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(3, idx.size());

  // original index of the point, so {2,0,1}
  EXPECT_EQ(2, idx.at(0));
  EXPECT_EQ(0, idx.at(1));
  EXPECT_EQ(1, idx.at(2));
}

TEST_F(Point2dManipulationsTest, 3PointsUnsorted2)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;

  Point2dManip::StdVectorEig2d points(3);
  points.at(0) = C;
  points.at(1) = A;
  points.at(2) = B;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(3, idx.size());

  EXPECT_EQ(1, idx.at(0));
  EXPECT_EQ(2, idx.at(1));
  EXPECT_EQ(0, idx.at(2));
}

// Testing with four points as used in four leg support polygons
TEST_F(Point2dManipulationsTest, 4PointsConvexSorted)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = A;
  points.at(1) = B;
  points.at(2) = C;
  points.at(3) = D;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(4, idx.size());

  EXPECT_EQ(0, idx.at(0));
  EXPECT_EQ(1, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
  EXPECT_EQ(3, idx.at(3));
}

TEST_F(Point2dManipulationsTest, 4PointsConvexUnsorted)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = B;
  points.at(1) = D;
  points.at(2) = C;
  points.at(3) = A;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(4, idx.size());

  EXPECT_EQ(3, idx.at(0));
  EXPECT_EQ(0, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
  EXPECT_EQ(1, idx.at(3));
}

TEST_F(Point2dManipulationsTest, 4PointsNonConvex)
{
  A << -1.0, -1.0;
  B <<  1.0, -2.0;
  C << -0.1,  0.1;
  D <<  0.0, -0.5;  // this point lies inside the convex hull of the others

  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = A;
  points.at(1) = B;
  points.at(2) = C;
  points.at(3) = D;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(3, idx.size()); // missing D

  EXPECT_EQ(0, idx.at(0));
  EXPECT_EQ(1, idx.at(1));
  EXPECT_EQ(2, idx.at(2));
}

TEST_F(Point2dManipulationsTest, 4PointsEqual)
{
  A << -1.0, -1.0;

  Point2dManip::StdVectorEig2d points(4);
  points.at(0) = A;
  points.at(1) = A;
  points.at(2) = A;
  points.at(3) = A;

  std::vector<size_t> idx = Point2dManip::BuildConvexHullCounterClockwise(points);
  EXPECT_EQ(1, idx.size()); // should only include the point once
  EXPECT_EQ(0, idx.at(0));
}


} // namespace utils
} // namespace xpp
