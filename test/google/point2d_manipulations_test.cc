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

TEST_F(Point2dManipulationsTest, LineCoeffNormalize)
{
  A << 0, 1;
  B << 2, 1;
  C << 0, 3;

  LineCoeff2d line_norm, line;
  line      = Point2dManip::LineCoeff(A,B, false);
  line_norm = Point2dManip::LineCoeff(A,B, true);

  // test distance to point
  double distance_to_C = line.p*C.x() + line.q*C.y() + line.r;
  double distance_to_C_norm = line_norm.p*C.x() + line_norm.q*C.y() + line_norm.r;

  EXPECT_NE(2.0, distance_to_C); // because line is not normalized
  EXPECT_DOUBLE_EQ(2.0, distance_to_C_norm);
}

TEST_F(Point2dManipulationsTest, LineCoefficients)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  LineCoeff2d line;
  // x-axis line (y=0)
  line = Point2dManip::LineCoeff(A,B);
  EXPECT_DOUBLE_EQ(0, line.p);
  EXPECT_DOUBLE_EQ(1, line.q);
  EXPECT_DOUBLE_EQ(0, line.r);

  // shifted y-axis line (x=1)
  line = Point2dManip::LineCoeff(B,C);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 0, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);

  // diagonal line to top right not normalized
  line = Point2dManip::LineCoeff(A,C,false);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ( 1, line.q);
  EXPECT_DOUBLE_EQ( 0, line.r);

  // diagonal line to top left not normalized
  line = Point2dManip::LineCoeff(B,D,false);
  EXPECT_DOUBLE_EQ(-1, line.p);
  EXPECT_DOUBLE_EQ(-1, line.q);
  EXPECT_DOUBLE_EQ( 1, line.r);
}

TEST_F(Point2dManipulationsTest, LineCoefficientsDistanceSign)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  // distance of point B and D to diagonal line A->C
  double distance_to_B, distance_to_D;
  xpp::utils::LineCoeff2d AC = xpp::utils::Point2dManip::LineCoeff(A,C);
  distance_to_B = AC.p*B.x() + AC.q*B.y() + AC.r;
  distance_to_D = AC.p*D.x() + AC.q*D.y() + AC.r;

  EXPECT_TRUE(distance_to_B < 0.0); // because B right of AC
  EXPECT_TRUE(distance_to_D > 0.0); // because D left  of AC


  // reverse the direction of the line, now from C to A
  xpp::utils::LineCoeff2d CA = xpp::utils::Point2dManip::LineCoeff(C,A);
  distance_to_B = CA.p*B.x() + CA.q*B.y() + CA.r;
  distance_to_D = CA.p*D.x() + CA.q*D.y() + CA.r;

  EXPECT_TRUE(distance_to_B > 0.0); // because B right of AC
  EXPECT_TRUE(distance_to_D < 0.0); // because D left  of AC
}

TEST_F(Point2dManipulationsTest, LineCoefficientsDistanceValue)
{
  A << 0, 0;
  B << 1, 0;
  C << 1, 1;
  D << 0, 1;

  // distance of point B and D to diagonal line A->C
  xpp::utils::LineCoeff2d AC = xpp::utils::Point2dManip::LineCoeff(A,C);
  double distance_to_B = AC.p*B.x() + AC.q*B.y() + AC.r;
  double distance_to_D = AC.p*D.x() + AC.q*D.y() + AC.r;

  EXPECT_DOUBLE_EQ(hypot(0.5, 0.5), std::fabs(distance_to_B));
  EXPECT_DOUBLE_EQ(distance_to_B, -distance_to_D); // both on opposite sides of line
}

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
