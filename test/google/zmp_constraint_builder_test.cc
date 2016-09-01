/*
 * gtest_zmp.cpp
 *
 *  Created on: May 23, 2014
 *      Author: awinkler
 */

#include <xpp/zmp/zmp_constraint_builder.h>
#include <xpp/hyq/support_polygon_container.h>
#include <xpp/zmp/com_spline.h>
#include <xpp/zmp/motion_factory.h>
#include <gtest/gtest.h>

namespace xpp {
namespace zmp {

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

  auto com_spline = MotionFactory::CreateComMotion(steps.size(),
                                                   SplineTimes(0.2, 0.3), true);

  ZmpConstraintBuilder builder;
  builder.Init(*com_spline, supp_polygons, 0.58);

  auto t_switch = builder.GetTimesDisjointSwitches();
  EXPECT_FLOAT_EQ(0.5, t_switch.at(0)); // RF -> LH
  EXPECT_FLOAT_EQ(0.9, t_switch.at(1)); // LF -> RH
}


} // namespace hyq
} // namespace xpp
