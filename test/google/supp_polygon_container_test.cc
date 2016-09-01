/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <gtest/gtest.h>
#include <xpp/hyq/support_polygon_container.h>

namespace xpp {
namespace hyq {


TEST(SuppPolygonContainerRefactor, FootholdID)
{
  auto start_stance = { Foothold( 0.35,  0.3, 0.0, LF),
                        Foothold( 0.35, -0.3, 0.0, RF),
                        Foothold(-0.35,  0.3, 0.0, LH),
                        Foothold(-0.35, -0.3, 0.0, RH)};

  auto steps =        { Foothold( 0.35+0.3,  0.3, 0.0, LF),
                        Foothold( 0.35+0.3, -0.3, 0.0, RF),
                        Foothold(-0.35+0.3,  0.3, 0.0, LH),
                        Foothold(-0.35+0.3, -0.3, 0.0, RH)};

  auto steps_zero =   { Foothold(0, 0, 0, LF),
                        Foothold(0, 0, 0, RF),
                        Foothold(0, 0, 0, LH),
                        Foothold(0, 0, 0, RH)};

  hyq::SupportPolygonContainer supp_polygons;
  supp_polygons.Init(start_stance, steps_zero, SupportPolygon::GetDefaultMargins());

  auto footholds = supp_polygons.GetFootholds();

  for (const auto& f : footholds)
    std::cout << f.id << std::endl;


  int i = 0;
  for (const auto& p : supp_polygons.GetSupportPolygons()) {
    std::cout << "\n\npolygon: " << i++ << std::endl;
    for (const auto& f : p.GetLines()/*footholds_conv_*/)
      std::cout << f.from.id << " , " << f.to.id << std::endl;
  }
}



class SuppPolygonContainerTest : public ::testing::Test {
protected:

  virtual void SetUp()
  {
    start_stance_[LH] = Foothold(-0.31,  0.37, 0.0, LH);
    start_stance_[LF] = Foothold( 0.33,  0.35, 0.0, LF);
    start_stance_[RH] = Foothold(-0.35, -0.33, 0.0, RH);
    start_stance_[RF] = Foothold( 0.37, -0.31, 0.0, RF);

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
  }

  SupportPolygon::VecFoothold f1_;
  SupportPolygonContainer start_to_final_supp_;

  LegDataMap<Foothold> start_stance_;
  LegDataMap<Foothold> final_stance_;

  SupportPolygon::VecFoothold f0_;
  SupportPolygonContainer start_to_f0_supp_;
};



TEST_F(SuppPolygonContainerTest, GetNumberOfSteps)
{
  EXPECT_EQ(4, start_to_final_supp_.GetNumberOfSteps());
}


TEST_F(SuppPolygonContainerTest, CreateSupportPolygons)
{
  std::vector<SupportPolygon> supp = start_to_final_supp_.CreateSupportPolygons(f0_);

  for (const xpp::hyq::SupportPolygon& p : supp)
    std::cout << "p: " << p;
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
  xpp::utils::StdVecEigen2d foothold_xy(4);
  for (auto& xy : foothold_xy)
    xy.setZero();


  start_to_final_supp_.SetFootholdsXY(foothold_xy);

  std::vector<SupportPolygon> supp = start_to_final_supp_.GetSupportPolygons();

  EXPECT_EQ(3, supp.at(0).GetFootholds().size()); // while lh swings to center
  EXPECT_EQ(3, supp.at(1).GetFootholds().size()); // while lf swings to center
  EXPECT_EQ(2, supp.at(2).GetFootholds().size()); // while rh swings to center (lh,lf at center)
  EXPECT_EQ(1, supp.at(3).GetFootholds().size()); // while rf swings to center

  for (const xpp::hyq::SupportPolygon& p : supp)
    std::cout << "p: " << p;
}

TEST_F(SuppPolygonContainerTest, GetFirstAndLastPolygon)
{
  // ordered from lowest x-y counterlockwise
  SupportPolygon start_stance = start_to_final_supp_.GetStartPolygon();
  EXPECT_EQ(4, start_stance.GetFootholds().size());
  EXPECT_EQ(start_stance_[RH], start_stance.GetFootholds().at(0));
  EXPECT_EQ(start_stance_[RF], start_stance.GetFootholds().at(1));
  EXPECT_EQ(start_stance_[LF], start_stance.GetFootholds().at(2));
  EXPECT_EQ(start_stance_[LH], start_stance.GetFootholds().at(3));


  SupportPolygon last_stance = start_to_final_supp_.GetFinalPolygon();
  EXPECT_EQ(4, last_stance.GetFootholds().size());
  EXPECT_EQ(final_stance_[RH], last_stance.GetFootholds().at(0));
  EXPECT_EQ(final_stance_[RF], last_stance.GetFootholds().at(1));
  EXPECT_EQ(final_stance_[LF], last_stance.GetFootholds().at(2));
  EXPECT_EQ(final_stance_[LH], last_stance.GetFootholds().at(3));
}


TEST_F(SuppPolygonContainerTest, SameFoothold)
{
  std::vector<SupportPolygon> supp = start_to_f0_supp_.GetSupportPolygons();
  EXPECT_EQ(4, supp.size()); // only one four leg support phase LF->RH

  EXPECT_EQ(3, supp.at(0).GetFootholds().size()); // while lh swings to center
  EXPECT_EQ(3, supp.at(1).GetFootholds().size()); // while lf swings to center
  EXPECT_EQ(2, supp.at(2).GetFootholds().size()); // while rh swings to center (lh,lf at center)
  EXPECT_EQ(1, supp.at(3).GetFootholds().size()); // while rf swings to center
}

} // namespace hyq
} // namespace xpp
