/**
 @file    zmp_contraint_builder_test.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Google unit tests for the ZmpConstraintBuilder class.
 */

#include <xpp/opt/zmp_constraint_builder.h>
#include <xpp/opt/com_spline.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/hyq/support_polygon_container.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

using namespace xpp::hyq;

TEST(ZmpConstraintBuilderTest, GetTimesDisjointSwitches)
{
  auto start_stance = { Foothold( 0.35,  0.3, 0.0, LF),
                        Foothold( 0.35, -0.3, 0.0, RF),
                        Foothold(-0.35,  0.3, 0.0, LH),
                        Foothold(-0.35, -0.3, 0.0, RH)};

  auto steps =        { Foothold( 0.0,  0.0, 0.0, RF),
                        Foothold( 0.0,  0.0, 0.0, LH),
                        Foothold( 0.0,  0.0, 0.0, LF),
                        Foothold( 0.0,  0.0, 0.0, RH)};

  SupportPolygonContainer supp_polygons;
  supp_polygons.Init(start_stance, steps, SupportPolygon::GetDefaultMargins());


  std::vector<xpp::hyq::LegID> legs;
  for (const auto& s : steps) {
    legs.push_back(static_cast<xpp::hyq::LegID>(s.id));
  }

  // create the fixed motion structure
  MotionStructure motion_structure;
  motion_structure.Init({}, legs, 0.3, 0.2, true, true);
  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases());


  ZmpConstraintBuilder builder;
  builder.Init(motion_structure, *com_motion, supp_polygons, 0.58, 0.1);

  auto t_switch = builder.GetTimesDisjointSwitches();
  EXPECT_FLOAT_EQ(0.5, t_switch.at(0)); // RF -> LH
  EXPECT_FLOAT_EQ(0.9, t_switch.at(1)); // LF -> RH


  std::cout << "GetTotalTIme: " << com_motion->GetTotalTime();
  std::cout << "\nt_switch_:\n";
  for (auto t : t_switch)
    std::cout << t << std::endl;

  std::cout << "\nt_constraint_:\n";
  auto t_constraint = builder.GetTimesForConstraitEvaluation(0.1, 0.2);
  for (auto t : t_constraint)
    std::cout << t << std::endl;
}


} // namespace hyq
} // namespace xpp
