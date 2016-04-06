/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/leg_data_map.h>
#include <xpp/hyq/foothold.h>
#include <xpp/utils/ellipse.h>

#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace hyq {

// A start and an end position for the splines. checking only boundary conditions
class HelpersTest : public ::testing::Test {

protected:
  virtual void SetUp()
  {
  }

};

TEST_F(HelpersTest, LinearSpliner)
{
  LegDataMap<Foothold> ldm;

  Foothold f_lh(-0.3, -0.3, 0.0, LH);
  Foothold f_lf(-0.3,  0.3, 0.0, LF);
  Foothold f_rh( 0.3, -0.3, 0.0, RH);
  Foothold f_rf( 0.3,  0.3, 0.0, RF);

  ldm[LH] = f_lh;
  ldm[LF] = f_lf;
  ldm[RH] = f_rh;
  ldm[RF] = f_rf;

  std::vector<Foothold> vec = ldm.ToVector();

  // always ordered lf, rf, lh, rh
  EXPECT_EQ(f_lf, vec.at(0));
  EXPECT_EQ(f_rf, vec.at(1));
  EXPECT_EQ(f_lh, vec.at(2));
  EXPECT_EQ(f_rh, vec.at(3));
}


TEST_F(HelpersTest, Ellipse)
{
  double x_center = 0.2;
  double y_center = 0.0;
  double x_axis_width = 0.3;
  double y_axis_width = 3.0;

  double x_radius = x_axis_width/2.0;
  double y_radius = y_axis_width/2.0;

  xpp::utils::Ellipse ellipse(x_axis_width, y_axis_width, x_center, y_center);

  double d_center_to_edge = ellipse.DistanceToEdge(x_center,y_center);
  EXPECT_DOUBLE_EQ(-1.0, d_center_to_edge);

  double pos_d_vertex_to_edge = ellipse.DistanceToEdge(x_center+x_radius,y_center);
  double neg_d_vertex_to_edge = ellipse.DistanceToEdge(x_center-x_radius,y_center);
  EXPECT_NEAR(0.0, pos_d_vertex_to_edge ,1e-8);
  EXPECT_NEAR(0.0, neg_d_vertex_to_edge ,1e-8);

  double d_outside_1 = ellipse.DistanceToEdge(x_center+x_radius+0.001,0.0);
  double d_outside_2 = ellipse.DistanceToEdge(x_center+0.001,         y_radius);
  double d_outside_3 = ellipse.DistanceToEdge(x_center+0.001,         y_radius);
  double d_outside_4 = ellipse.DistanceToEdge(x_center-x_radius-0.001,0.0);

  EXPECT_GT(d_outside_1, 0.0);
  EXPECT_GT(d_outside_2, 0.0);
  EXPECT_GT(d_outside_3, 0.0);
  EXPECT_GT(d_outside_4, 0.0);
}






} // namespace hyq
} // namespace xpp
