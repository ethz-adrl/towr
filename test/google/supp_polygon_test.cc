/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/support_polygon.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;

namespace xpp {
namespace hyq {


class SuppPolygonTest : public ::testing::Test {
protected:
  virtual void SetUp()
  {

    // rh foot located at origin minus offset
    Foothold rh(0.0-offset_, 0.0, 0.0, RH);
    Foothold rf(1.0-offset_, 0.0, 0.0, RF);
    Foothold lf(1.0-offset_, 1.0, 0.0, LF);
    Foothold lh(0.0-offset_, 1.0, 0.0, LH);

    // bottom right (ordered)
    f_bottom_right.push_back(lh);
    f_bottom_right.push_back(rh);
    f_bottom_right.push_back(rf);

    // top left (not ordered)
    f_top_left.push_back(lh);
    f_top_left.push_back(lf);
    f_top_left.push_back(rf);

    // top right (not ordered)
    f_top_right.push_back(lf);
    f_top_right.push_back(rf);
    f_top_right.push_back(rh);


    // all (ordered)
    f_4_ordered.push_back(lh);
    f_4_ordered.push_back(rh);
    f_4_ordered.push_back(rf);
    f_4_ordered.push_back(lf);

    // all (not ordered)
    f_4_not_ordered.push_back(lh);
    f_4_not_ordered.push_back(rh);
    f_4_not_ordered.push_back(lf);
    f_4_not_ordered.push_back(rf);

    // all but nonconvex M in middle
    f_all_non_conv.push_back(lh);
    f_all_non_conv.push_back(rh);
    f_all_non_conv.push_back(lf);
    f_all_non_conv.push_back(rf);
    f_all_non_conv.push_back(Foothold(0.5, 0.5, 0.0, RF));


    margins_[DIAG]  = 0.08;
    margins_[FRONT] = 0.1;
    margins_[HIND]  = 0.12;
    margins_[SIDE]  = 0.14;
  }


  SupportPolygon::VecFoothold f_bottom_right;
  SupportPolygon::VecFoothold f_top_left;
  SupportPolygon::VecFoothold f_top_right;
  SupportPolygon::VecFoothold f_4_ordered;
  SupportPolygon::VecFoothold f_4_not_ordered;
  SupportPolygon::VecFoothold f_all_non_conv;

  // default position, x direction facing left, y facing down
  const double offset_ = 0.3;

  MarginValues margins_;
};


TEST_F(SuppPolygonTest, CombineSupportPolygons)
{
  SupportPolygon supp1 = SupportPolygon(f_bottom_right, margins_);
  SupportPolygon supp2 = SupportPolygon(f_top_left, margins_);

  SupportPolygon combined = SupportPolygon::CombineSupportPolygons(supp1, supp2);

  EXPECT_EQ(4, combined.GetFootholds().size());
  EXPECT_EQ(f_4_ordered, combined.GetFootholds());
  EXPECT_EQ(supp1.GetMargins(), combined.GetMargins());
}


TEST_F(SuppPolygonTest, CombineSupportPolygonsSame)
{
  SupportPolygon supp1 = SupportPolygon(f_bottom_right, margins_);
  SupportPolygon combined = SupportPolygon::CombineSupportPolygons(supp1, supp1);

  EXPECT_EQ(f_bottom_right, combined.GetFootholds());
}


TEST_F(SuppPolygonTest, IsZmpInsideSuppPolygon4Contacts)
{
  using Vector2d = Eigen::Vector2d;

  SupportPolygon::VecFoothold stance;
  stance.push_back(Foothold(-0.33,  0.31, 0.0, hyq::LH));
  stance.push_back(Foothold( 0.33,  0.31, 0.0, hyq::LF));
  stance.push_back(Foothold(-0.33, -0.31, 0.0, hyq::RH));
  stance.push_back(Foothold( 0.33, -0.31, 0.0, hyq::RF));

  hyq::MarginValues margins;
  margins[hyq::FRONT] = 0.10;
  margins[hyq::HIND]  = 0.10;
  margins[hyq::SIDE]  = 0.10;
  margins[hyq::DIAG]  = 0.06;
  SupportPolygon supp(stance, margins);
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

TEST_F(SuppPolygonTest, IsZmpInsideSuppPolygon3Contacts)
{
  using Vector2d = Eigen::Vector2d;

  SupportPolygon::VecFoothold stance;
  stance.push_back(Foothold( 0.33,  0.31, 0.0, hyq::LF));
  stance.push_back(Foothold(-0.33, -0.31, 0.0, hyq::RH));
  stance.push_back(Foothold( 0.33, -0.31, 0.0, hyq::RF));

  hyq::MarginValues margins;
  margins[hyq::FRONT] = 0.10;
  margins[hyq::HIND]  = 0.10;
  margins[hyq::SIDE]  = 0.10;
  margins[hyq::DIAG]  = 0.06;
  SupportPolygon supp(stance, margins);

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

//TEST_F(SuppPolygonTest, CalcLinesTopRight)
//{
//  SupportPolygon supp = SupportPolygon(f_top_right, margins_);
//  SupportPolygon::VecSuppLine lines = supp.GetLines();
//
//  // expect three lines
//  EXPECT_EQ(3, lines.size());
//  // RH->RF: 0*x + 1*y + 0 = 0
//  EXPECT_FLOAT_EQ( 0, lines.at(0).coeff.p);
//  EXPECT_FLOAT_EQ( 1, lines.at(0).coeff.q);
//  EXPECT_FLOAT_EQ( 0, lines.at(0).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[SIDE], lines.at(0).s_margin);
//  // RF->LF: -1*x + 0*y + 1 = 0
//  EXPECT_FLOAT_EQ(-1, lines.at(1).coeff.p);
//  EXPECT_FLOAT_EQ( 0, lines.at(1).coeff.q);
//  EXPECT_FLOAT_EQ( 1-offset_, lines.at(1).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[FRONT], lines.at(1).s_margin);
//  // C->A: 1*x - 1*y + 0 = 0    // diagonal normalized
//  EXPECT_FLOAT_EQ( 1/sqrt(2), lines.at(2).coeff.p);
//  EXPECT_FLOAT_EQ(-1/sqrt(2), lines.at(2).coeff.q);
//  EXPECT_FLOAT_EQ( +offset_/sqrt(2.), lines.at(2).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[DIAG], lines.at(2).s_margin);
//}


//TEST_F(SuppPolygonTest, FourLegSuppNotOrdered)
//{
//  SupportPolygon supp = SupportPolygon(f_4_not_ordered, margins_);
//  SupportPolygon::VecSuppLine lines = supp.GetLines();
//
//  EXPECT_EQ(4, lines.size());
//  // RH->RF: 0*x + 1*y + 0 = 0
//  EXPECT_FLOAT_EQ( 0, lines.at(0).coeff.p);
//  EXPECT_FLOAT_EQ( 1, lines.at(0).coeff.q);
//  EXPECT_FLOAT_EQ( 0, lines.at(0).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[SIDE], lines.at(0).s_margin);
//  // RF->LF: -1*x + 0*y + 1 = 0
//  EXPECT_FLOAT_EQ(-1, lines.at(1).coeff.p);
//  EXPECT_FLOAT_EQ( 0, lines.at(1).coeff.q);
//  EXPECT_FLOAT_EQ( 1-offset_, lines.at(1).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[FRONT], lines.at(1).s_margin);
//  // LF->LH: 0*x + -1*y + 1 = 0
//  EXPECT_FLOAT_EQ( 0, lines.at(2).coeff.p);
//  EXPECT_FLOAT_EQ(-1, lines.at(2).coeff.q);
//  EXPECT_FLOAT_EQ( 1, lines.at(2).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[SIDE], lines.at(2).s_margin);
//  // LH->RH:  1*x + 0*y + 0 = 0
//  EXPECT_FLOAT_EQ( 1, lines.at(3).coeff.p);
//  EXPECT_FLOAT_EQ( 0, lines.at(3).coeff.q);
//  EXPECT_FLOAT_EQ( +offset_, lines.at(3).coeff.r);
//  EXPECT_FLOAT_EQ( margins_[HIND], lines.at(3).s_margin);
//}






} // namespace hyq
} // namespace xpp
