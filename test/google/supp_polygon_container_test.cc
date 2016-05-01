/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/continuous_spline_container.h>
#include <iostream>

#define prt(x) std::cout << #x << " = " << x << std::endl;
#define prt(x)

namespace xpp {
namespace hyq {


class SuppPolygonContainerTest : public ::testing::Test {
protected:

  typedef xpp::zmp::SplineContainer SplineContainer;

  virtual void SetUp()
  {
    s_lh.x() = -0.3;
    s_lh.y() =  0;

    s_lf.x() = -0.3;
    s_lf.y() =  0.6;

    s_rh.x() = 0.3;
    s_rh.y() = 0;

    s_rf.x() = 0.3;
    s_rf.y() = 0.6;

    start_stance_[LH] = Foothold(-0.3, -0.3, 0.0, LH);
    start_stance_[LF] = Foothold(-0.3,  0.3, 0.0, LF);
    start_stance_[RH] = Foothold( 0.3, -0.3, 0.0, RH);
    start_stance_[RF] = Foothold( 0.3,  0.3, 0.0, RF);

    final_stance_[LH] = Foothold(s_lh.x(), s_lh.y(), 0.0, LH);
    final_stance_[LF] = Foothold(s_lf.x(), s_lf.y(), 0.0, LF);
    final_stance_[RH] = Foothold(s_rh.x(), s_rh.y(), 0.0, RH);
    final_stance_[RF] = Foothold(s_rf.x(), s_rf.y(), 0.0, RF);

    f_.push_back(final_stance_[LH]);
    f_.push_back(final_stance_[LF]);
    f_.push_back(final_stance_[RH]);
    f_.push_back(final_stance_[RF]);

    f0_.push_back(Foothold(0.0, 0.0, 0.0, LH));
    f0_.push_back(Foothold(0.0, 0.0, 0.0, LF));
    f0_.push_back(Foothold(0.0, 0.0, 0.0, RH));
    f0_.push_back(Foothold(0.0, 0.0, 0.0, RF));

    margins_[FRONT] = 0.0;
    margins_[HIND]  = 0.0;
    margins_[SIDE]  = 0.0;
    margins_[DIAG]  = 0.0;


    cont_.Init(start_stance_.ToVector(), f_, margins_);
    cont0_.Init(start_stance_.ToVector(), f0_, margins_);

    double t_stance = 0.1;
    double t_swing = 0.6;
    double t_stance_initial = 2.0;
    double t_stance_final = 1.0;
    std::vector<LegID> step_sequence = {f_.at(0).leg, f_.at(1).leg,
                                        f_.at(2).leg, f_.at(3).leg};

    splines_ = SplineContainer::ConstructSplineSequence(step_sequence, t_stance, t_swing, t_stance_initial, t_stance_final);

  }

  // points arranged in positive square starting at origin
  Eigen::Vector2d s_lh;
  Eigen::Vector2d s_rh;
  Eigen::Vector2d s_rf;
  Eigen::Vector2d s_lf;

  SupportPolygon::VecFoothold f_;
  SupportPolygon::VecFoothold f0_;
  LegDataMap<Foothold> start_stance_;
  LegDataMap<Foothold> final_stance_;
  MarginValues margins_;

  SupportPolygonContainer cont_;
  SupportPolygonContainer cont0_;
  SplineContainer::VecSpline splines_;

};


TEST_F(SuppPolygonContainerTest, CreateSupportPolygons)
{
  std::vector<SupportPolygon> supp = cont_.GetSupportPolygons();
  EXPECT_EQ(4, supp.size());

  std::vector<SupportPolygon> supp_4ls;

  for (const xpp::zmp::ZmpSpline& s : splines_) {
    if (s.GetType() == xpp::zmp::Intermediate4lsSpline) {
      int next_step = s.GetNextPlannedStep();
      supp_4ls.push_back(SupportPolygon::CombineSupportPolygons(supp.at(next_step),
                                                                supp.at(next_step-1)));
    }
  }

  EXPECT_EQ(1, supp_4ls.size()); // only one four leg support phase LF->RH
  EXPECT_EQ(final_stance_[LH], supp_4ls.at(0).footholds_conv_.at(0));
  EXPECT_EQ(start_stance_[RH], supp_4ls.at(0).footholds_conv_.at(1));
  EXPECT_EQ(start_stance_[RF], supp_4ls.at(0).footholds_conv_.at(2));
  EXPECT_EQ(final_stance_[LF], supp_4ls.at(0).footholds_conv_.at(3));
}


TEST_F(SuppPolygonContainerTest, GetFirstAndLastPolygon)
{
  SupportPolygon start_stance = cont_.GetStartPolygon();
  EXPECT_EQ(4, start_stance.footholds_conv_.size());
  EXPECT_EQ(start_stance_[LH], start_stance.footholds_conv_.at(0));
  EXPECT_EQ(start_stance_[RH], start_stance.footholds_conv_.at(1));
  EXPECT_EQ(start_stance_[RF], start_stance.footholds_conv_.at(2));
  EXPECT_EQ(start_stance_[LF], start_stance.footholds_conv_.at(3));


  SupportPolygon last_stance = cont_.GetFinalPolygon();
  EXPECT_EQ(4, last_stance.footholds_conv_.size());
  EXPECT_NE(f_, last_stance.footholds_conv_);
  EXPECT_EQ(final_stance_[LH], last_stance.footholds_conv_.at(0));
  EXPECT_EQ(final_stance_[RH], last_stance.footholds_conv_.at(1));
  EXPECT_EQ(final_stance_[RF], last_stance.footholds_conv_.at(2));
  EXPECT_EQ(final_stance_[LF], last_stance.footholds_conv_.at(3));
}


TEST_F(SuppPolygonContainerTest, SameFoothold)
{
  std::vector<SupportPolygon> supp = cont0_.GetSupportPolygons();
  EXPECT_EQ(4, supp.size()); // only one four leg support phase LF->RH

  EXPECT_EQ(3, supp.at(0).footholds_conv_.size()); // lh swings
  EXPECT_EQ(3, supp.at(1).footholds_conv_.size()); // lf swings
  EXPECT_EQ(2, supp.at(2).footholds_conv_.size()); // rh swings
  EXPECT_EQ(1, supp.at(3).footholds_conv_.size()); // rf swings

  prt(supp.at(0));
  prt(supp.at(1));

  SupportPolygon supp4 = SupportPolygon::CombineSupportPolygons(supp.at(2), supp.at(1));
  prt(supp4);
  for (const Foothold& f : supp4.footholds_) {
    prt(f);
  }

  prt(supp.at(2));
  prt(supp.at(3));
}


} // namespace hyq
} // namespace xpp
