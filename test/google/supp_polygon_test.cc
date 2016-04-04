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
    // points arranged in positive square starting at origin
    Eigen::Vector2d A;
    Eigen::Vector2d B;
    Eigen::Vector2d C;
    Eigen::Vector2d D;
    Eigen::Vector2d M;

    A.x() = 0;
    A.y() = 0;

    B.x() = 1;
    B.y() = 0;

    C.x() = 1;
    C.y() = 1;

    D.x() = 0;
    D.y() = 1;

    M.x() = 0.5;
    M.y() = 0.5;


    // bottom right (ordered)
    f_bottom_right.push_back(Foothold(A.x(), A.y(), 0.0, LH));
    f_bottom_right.push_back(Foothold(B.x(), B.y(), 0.0, RH));
    f_bottom_right.push_back(Foothold(C.x(), C.y(), 0.0, RF));

    // top left (not ordered)
    f_top_left.push_back(Foothold(A.x(), A.y(), 0.0, LH));
    f_top_left.push_back(Foothold(D.x(), D.y(), 0.0, LF));
    f_top_left.push_back(Foothold(C.x(), C.y(), 0.0, RF));

    // all (ordered)
    f_4_ordered.push_back(Foothold(A.x(), A.y(), 0.0, LH));
    f_4_ordered.push_back(Foothold(B.x(), B.y(), 0.0, RH));
    f_4_ordered.push_back(Foothold(C.x(), C.y(), 0.0, RF));
    f_4_ordered.push_back(Foothold(D.x(), D.y(), 0.0, LF));

    // all (not ordered)
    f_4_not_ordered.push_back(Foothold(A.x(), A.y(), 0.0, LH));
    f_4_not_ordered.push_back(Foothold(B.x(), B.y(), 0.0, RH));
    f_4_not_ordered.push_back(Foothold(D.x(), D.y(), 0.0, LF));
    f_4_not_ordered.push_back(Foothold(C.x(), C.y(), 0.0, RF));

    // all but nonconvex M in middle
    f_all_non_conv.push_back(Foothold(A.x(), A.y(), 0.0, LH));
    f_all_non_conv.push_back(Foothold(B.x(), B.y(), 0.0, RH));
    f_all_non_conv.push_back(Foothold(D.x(), D.y(), 0.0, LF));
    f_all_non_conv.push_back(Foothold(C.x(), C.y(), 0.0, RF));
    f_all_non_conv.push_back(Foothold(M.x(), M.y(), 0.0, RF));


    margins_[FRONT] = 0.0;
    margins_[HIND]  = 0.0;
    margins_[SIDE]  = 0.0;
    margins_[DIAG]  = 0.0;

  }


  SupportPolygon::VecFoothold f_bottom_right;
  SupportPolygon::VecFoothold f_top_left;
  SupportPolygon::VecFoothold f_4_ordered;
  SupportPolygon::VecFoothold f_4_not_ordered;
  SupportPolygon::VecFoothold f_all_non_conv;

  MarginValues margins_;
};


TEST_F(SuppPolygonTest, OrderCounterClockwise)
{
  SupportPolygon poly;
  poly = SupportPolygon(margins_, f_bottom_right);
  EXPECT_EQ(f_bottom_right.size(), poly.footholds_conv_.size());
  EXPECT_EQ(f_bottom_right, poly.footholds_);
  EXPECT_EQ(f_bottom_right, poly.footholds_conv_);

  poly = SupportPolygon(margins_, f_top_left);
  EXPECT_EQ(f_top_left.size(), poly.footholds_conv_.size());
  EXPECT_EQ(f_top_left, poly.footholds_);
  EXPECT_NE(f_top_left, poly.footholds_conv_);

  poly = SupportPolygon(margins_, f_4_not_ordered);
  EXPECT_EQ(f_4_not_ordered.size(), poly.footholds_conv_.size());
  EXPECT_EQ(f_4_not_ordered, poly.footholds_);
  EXPECT_NE(f_4_not_ordered, poly.footholds_conv_);

  poly = SupportPolygon(margins_, f_all_non_conv);
  EXPECT_EQ(f_all_non_conv.size()-1, poly.footholds_conv_.size());
  EXPECT_EQ(f_all_non_conv, poly.footholds_);
  EXPECT_NE(f_all_non_conv, poly.footholds_conv_);
}


TEST_F(SuppPolygonTest, CombineSupportPolygons)
{
  SupportPolygon supp1 = SupportPolygon(margins_, f_bottom_right);
  SupportPolygon supp2 = SupportPolygon(margins_, f_top_left);

  SupportPolygon combined = SupportPolygon::CombineSupportPolygons(supp1, supp2);

  EXPECT_EQ(4, combined.footholds_conv_.size());
  EXPECT_EQ(f_4_ordered, combined.footholds_conv_);
  EXPECT_NE(f_4_ordered, combined.footholds_);
  EXPECT_EQ(supp1.margins_, combined.margins_);
}


TEST_F(SuppPolygonTest, CombineSupportPolygonsSame)
{
  SupportPolygon supp1 = SupportPolygon(margins_, f_bottom_right);
  SupportPolygon combined = SupportPolygon::CombineSupportPolygons(supp1, supp1);

  EXPECT_EQ(f_bottom_right, combined.footholds_conv_);
  EXPECT_NE(f_bottom_right, combined.footholds_);
}



} // namespace hyq
} // namespace xpp
