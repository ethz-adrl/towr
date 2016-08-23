/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/zmp/zmp_constraint_builder.h>

#include <gtest/gtest.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace zmp {


class ZmpConstraintTest : public ::testing::Test {

public:
  typedef ZmpConstraintBuilder::MatVec MatVec;
  typedef ZmpConstraintBuilder::Vector2d Vector2d;
  typedef xpp::hyq::SupportPolygon SupportPolygon;
  typedef xpp::hyq::Foothold Foothold;

protected:
  virtual void SetUp()
  {
    cont_spline_container_.Init(Eigen::Vector2d::Zero(),
                                Eigen::Vector2d::Zero(),
                                1,
                                SplineTimes(t_swing, t_stance_initial));

    zmp_constaint_.Init(cont_spline_container_, walking_height);

    supp_poly_margins[hyq::FRONT] = 0.10;
    supp_poly_margins[hyq::HIND]  = 0.10;
    supp_poly_margins[hyq::SIDE]  = 0.10;
    supp_poly_margins[hyq::DIAG]  = 0.06; // controls sidesway motion 0.8 even better
  }

  ComSpline6 spline_container_;
  ComSpline4 cont_spline_container_;
  double t_stance_initial = 2.0;
  double t_swing = 0.5;
  double walking_height = 0.58;

  hyq::MarginValues supp_poly_margins;

  ZmpConstraintBuilder zmp_constaint_;
};


TEST_F(ZmpConstraintTest, IsZmpInsideSuppPolygon4Contacts)
{
  SupportPolygon::VecFoothold stance;
  stance.push_back(Foothold(-0.33,  0.31, 0.0, hyq::LH));
  stance.push_back(Foothold( 0.33,  0.31, 0.0, hyq::LF));
  stance.push_back(Foothold(-0.33, -0.31, 0.0, hyq::RH));
  stance.push_back(Foothold( 0.33, -0.31, 0.0, hyq::RF));

  SupportPolygon supp(stance, supp_poly_margins);
  EXPECT_TRUE(supp.IsPointInside(Vector2d( 0.0, 0.0)));

  // at x and y borders
  EXPECT_TRUE(supp.IsPointInside(Vector2d( 0.23, 0.0)));
  EXPECT_TRUE(supp.IsPointInside(Vector2d(-0.23, 0.0)));
  EXPECT_TRUE(supp.IsPointInside(Vector2d(  0.0, 0.21)));
  EXPECT_TRUE(supp.IsPointInside(Vector2d(  0.0,-0.21)));

  // over x and y borders
  EXPECT_FALSE(supp.IsPointInside(Vector2d( 0.24, 0.0)));
  EXPECT_FALSE(supp.IsPointInside(Vector2d(-0.24, 0.0)));
  EXPECT_FALSE(supp.IsPointInside(Vector2d(  0.0, 0.22)));
  EXPECT_FALSE(supp.IsPointInside(Vector2d(  0.0,-0.22)));
}

TEST_F(ZmpConstraintTest, IsZmpInsideSuppPolygon3Contacts)
{
  SupportPolygon::VecFoothold stance;
  stance.push_back(Foothold( 0.33,  0.31, 0.0, hyq::LF));
  stance.push_back(Foothold(-0.33, -0.31, 0.0, hyq::RH));
  stance.push_back(Foothold( 0.33, -0.31, 0.0, hyq::RF));

  SupportPolygon supp(stance, supp_poly_margins);

  // point at diagonal is not inside, since support polygon shrunk by stability margin
  EXPECT_FALSE(supp.IsPointInside(Vector2d( 0.0, 0.0)));

  // zmp forward works, because left hind leg not touching. zmp backwards fails
  EXPECT_TRUE(supp.IsPointInside(Vector2d(  0.1, 0.0)));
  EXPECT_FALSE(supp.IsPointInside(Vector2d(-0.1, 0.0)));

  // zmp right works, because left hind leg not touching. zmp right fails
  EXPECT_TRUE(supp.IsPointInside(Vector2d( 0.0,-0.1)));
  EXPECT_FALSE(supp.IsPointInside(Vector2d(0.0, 0.1)));

  // moving towards front right workd, towards left hind fails
  EXPECT_TRUE(supp.IsPointInside(Vector2d(  0.1,-0.1)));
  EXPECT_FALSE(supp.IsPointInside(Vector2d(-0.1, 0.1)));
}



} // namespace hyq
} // namespace xpp
