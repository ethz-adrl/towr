/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/support_polygon.h>
#include <xpp/utils/point2d_manipulations.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace utils {


class SuppPolygonTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    A.x() = 0;
    A.y() = 0;

    B.x() = 1;
    B.y() = 0;

    C.x() = 1;
    C.y() = 1;

    D.x() = 0;
    D.y() = 1;
  }

  // points arranged in positive square starting at origin
  Eigen::Vector2d A;
  Eigen::Vector2d B;
  Eigen::Vector2d C;
  Eigen::Vector2d D;
};


TEST_F(SuppPolygonTest, LineConstraint)
{
  LineCoeff2d AC;
  AC = Point2dManip::LineCoeff(A,C);

  double distance_to_B = AC.p*B.x() + AC.q*B.y() + AC.r;
  double distance_to_D = AC.p*D.x() + AC.q*D.y() + AC.r;

  EXPECT_TRUE(distance_to_B < 0.0); // because B right of AC
  EXPECT_TRUE(distance_to_D > 0.0); // because D left  of AC

  EXPECT_NEAR(hypot(0.5, 0.5), std::fabs(distance_to_B), 0.001);
  EXPECT_DOUBLE_EQ(std::fabs(distance_to_B), std::fabs(distance_to_D));
}



} // namespace hyq
} // namespace xpp
