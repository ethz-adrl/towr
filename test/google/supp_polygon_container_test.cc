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
//#define prt(x)

namespace xpp {
namespace hyq {


class SuppPolygonContainerTest : public ::testing::Test {
protected:

  typedef xpp::zmp::SplineContainer SplineContainer;
  typedef xpp::zmp::SplineContainer::VecSpline VecSpline;

  virtual void SetUp()
  {
    start_stance_[LH] = Foothold(-0.31,  0.37, 0.0, LH);
    start_stance_[LF] = Foothold( 0.33,  0.35, 0.0, LF);
    start_stance_[RH] = Foothold(-0.35, -0.33, 0.0, RH);
    start_stance_[RF] = Foothold( 0.37, -0.31, 0.0, RF);

    double step_length = 0.25;
    Eigen::Vector3d step;
    step << 0.25, 0.05, 0.0;
    for (LegID leg : LegIDArray) {
      final_stance_[leg] = start_stance_[leg];
      final_stance_[leg].p += step;
    }

    // decide step sequence
    f1_.push_back(Foothold(final_stance_[LH].p.x(), final_stance_[LH].p.y(), 0.0, LH));
    f1_.push_back(Foothold(final_stance_[LF].p.x(), final_stance_[LF].p.y(), 0.0, LF));
    f1_.push_back(Foothold(final_stance_[RH].p.x(), final_stance_[RH].p.y(), 0.0, RH));
    f1_.push_back(Foothold(final_stance_[RF].p.x(), final_stance_[RF].p.y(), 0.0, RF));
    start_to_final_supp_.Init(start_stance_.ToVector(), f1_);


    f0_.push_back(Foothold(0.0, 0.0, 0.0, LH));
    f0_.push_back(Foothold(0.0, 0.0, 0.0, LF));
    f0_.push_back(Foothold(0.0, 0.0, 0.0, RH));
    f0_.push_back(Foothold(0.0, 0.0, 0.0, RF));
    start_to_f0_supp_.Init(start_stance_.ToVector(), f0_);



    double t_stance = 0.1;
    double t_swing = 0.6;
    double t_stance_initial = 2.0;
    double t_stance_final = 1.0;
    std::vector<LegID> step_sequence = {LH, LF, RH, RF};

    splines_ = SplineContainer::ConstructSplineSequence(step_sequence,
                                                        t_stance,
                                                        t_swing,
                                                        t_stance_initial,
                                                        t_stance_final);

  }

  virtual int GetInitial4lsSplines(const VecSpline& splines) const {
    int n = 0;
    for (const xpp::zmp::ZmpSpline& s: splines)
      if (s.GetType() == xpp::zmp::Initial4lsSpline)
        n++;

    return n;
  }

  SupportPolygon::VecFoothold f1_;
  SupportPolygonContainer start_to_final_supp_;

  LegDataMap<Foothold> start_stance_;
  LegDataMap<Foothold> final_stance_;

  SupportPolygon::VecFoothold f0_;
  SupportPolygonContainer start_to_f0_supp_;
  SplineContainer::VecSpline splines_;

};


TEST_F(SuppPolygonContainerTest, CreateSupportPolygonsWith4LS)
{
  std::vector<xpp::hyq::SupportPolygon> supp_all;
  supp_all = start_to_final_supp_.CreateSupportPolygonsWith4LS(splines_);

  for (const xpp::hyq::SupportPolygon& p : supp_all)
    prt(p);

  int inital_splines = GetInitial4lsSplines(splines_);

  int id = 0; // number of initial four leg support splines
  SCOPED_TRACE("id" + id);
  for (int j=0; j<inital_splines; ++j)
    EXPECT_EQ(4, supp_all.at(id++).footholds_conv_.size()); // initial four leg support phases

  // possibly more 4leg support splines
  EXPECT_EQ(3, supp_all.at(id++).footholds_conv_.size()); // step 0
  EXPECT_EQ(3, supp_all.at(id++).footholds_conv_.size()); // step 1
  EXPECT_EQ(4, supp_all.at(id++).footholds_conv_.size()); // 4ls
  EXPECT_EQ(3, supp_all.at(id++).footholds_conv_.size()); // step 2
  EXPECT_EQ(3, supp_all.at(id++).footholds_conv_.size()); // step 3
  EXPECT_EQ(4, supp_all.at(id++).footholds_conv_.size()); // final 4ls
}


TEST_F(SuppPolygonContainerTest, GetNumberOfSteps)
{
  EXPECT_EQ(4, start_to_final_supp_.GetNumberOfSteps());
}


TEST_F(SuppPolygonContainerTest, CreateSupportPolygons)
{
  std::vector<SupportPolygon> supp = start_to_final_supp_.CreateSupportPolygons(f0_);

  for (const xpp::hyq::SupportPolygon& p : supp)
    prt(p);
}


TEST_F(SuppPolygonContainerTest, GetStanceAfter)
{
  for (int i=0; i<start_to_final_supp_.GetNumberOfSteps(); ++i) {
    std::cout << "\n\n step: " << i << ":\n";
    for (int j=0; j<4; ++j) {
      std::cout << start_to_final_supp_.GetStanceAfter(i).at(j) << std::endl;
    }
  }
}


TEST_F(SuppPolygonContainerTest, SetFootholdsXY)
{
  start_to_final_supp_.SetFootholdsXY(0, 0.0, 0.0);
  start_to_final_supp_.SetFootholdsXY(1, 0.0, 0.0);
  start_to_final_supp_.SetFootholdsXY(2, 0.0, 0.0);
  start_to_final_supp_.SetFootholdsXY(3, 0.0, 0.0);

  std::vector<SupportPolygon> supp = start_to_final_supp_.GetSupportPolygons();

  EXPECT_EQ(3, supp.at(0).footholds_conv_.size()); // while lh swings to center
  EXPECT_EQ(3, supp.at(1).footholds_conv_.size()); // while lf swings to center
  EXPECT_EQ(2, supp.at(2).footholds_conv_.size()); // while rh swings to center (lh,lf at center)
  EXPECT_EQ(1, supp.at(3).footholds_conv_.size()); // while rf swings to center

  for (const xpp::hyq::SupportPolygon& p : supp)
    prt(p);
}



TEST_F(SuppPolygonContainerTest, CombineSupportPolygons)
{
  std::vector<SupportPolygon> supp = start_to_final_supp_.GetSupportPolygons();
  EXPECT_EQ(4, supp.size());

  std::vector<SupportPolygon> supp_4ls;

  for (const xpp::zmp::ZmpSpline& s : splines_) {
    if (s.GetType() == xpp::zmp::Intermediate4lsSpline) {
      int next_step = s.GetNextPlannedStep();
      supp_4ls.push_back(SupportPolygon::CombineSupportPolygons(supp.at(next_step),
                                                                supp.at(next_step-1)));
    }
  }

  // only one four leg support phase LF->RH
  // points sorted counterclockwise starting at minimum x and y point
  EXPECT_EQ(1, supp_4ls.size());
  EXPECT_EQ(start_stance_[RH], supp_4ls.at(0).footholds_conv_.at(0));
  EXPECT_EQ(start_stance_[RF], supp_4ls.at(0).footholds_conv_.at(1));
  EXPECT_EQ(final_stance_[LF], supp_4ls.at(0).footholds_conv_.at(2));
  EXPECT_EQ(final_stance_[LH], supp_4ls.at(0).footholds_conv_.at(3));
}


TEST_F(SuppPolygonContainerTest, GetFirstAndLastPolygon)
{
  // ordered from lowest x-y counterlockwise
  SupportPolygon start_stance = start_to_final_supp_.GetStartPolygon();
  EXPECT_EQ(4, start_stance.footholds_conv_.size());
  EXPECT_EQ(start_stance_[RH], start_stance.footholds_conv_.at(0));
  EXPECT_EQ(start_stance_[RF], start_stance.footholds_conv_.at(1));
  EXPECT_EQ(start_stance_[LF], start_stance.footholds_conv_.at(2));
  EXPECT_EQ(start_stance_[LH], start_stance.footholds_conv_.at(3));


  SupportPolygon last_stance = start_to_final_supp_.GetFinalPolygon();
  EXPECT_EQ(4, last_stance.footholds_conv_.size());
  EXPECT_EQ(final_stance_[RH], last_stance.footholds_conv_.at(0));
  EXPECT_EQ(final_stance_[RF], last_stance.footholds_conv_.at(1));
  EXPECT_EQ(final_stance_[LF], last_stance.footholds_conv_.at(2));
  EXPECT_EQ(final_stance_[LH], last_stance.footholds_conv_.at(3));
}


TEST_F(SuppPolygonContainerTest, SameFoothold)
{
  std::vector<SupportPolygon> supp = start_to_f0_supp_.GetSupportPolygons();
  EXPECT_EQ(4, supp.size()); // only one four leg support phase LF->RH

  EXPECT_EQ(3, supp.at(0).footholds_conv_.size()); // while lh swings to center
  EXPECT_EQ(3, supp.at(1).footholds_conv_.size()); // while lf swings to center
  EXPECT_EQ(2, supp.at(2).footholds_conv_.size()); // while rh swings to center (lh,lf at center)
  EXPECT_EQ(1, supp.at(3).footholds_conv_.size()); // while rf swings to center
}


} // namespace hyq
} // namespace xpp
