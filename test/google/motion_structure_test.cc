/**
 @file    stance_feet_calculator_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jun 6, 2016
 @brief   Brief description
 */

#include <xpp/opt/motion_structure.h>
#include <xpp/hyq/foothold.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

using namespace xpp::hyq;

TEST(MotionStructureTest, BuildPhasesWithContact)
{
  auto start_stance = { Foothold( 0.35,  0.3, 0.0, LF),
                        Foothold( 0.35, -0.3, 0.0, RF),
                        Foothold(-0.35,  0.3, 0.0, LH),
                        Foothold(-0.35, -0.3, 0.0, RH)};

  auto steps =        { LH, LF, RH, RF};

  // create the fixed motion structure
  MotionStructure motion_structure;
  motion_structure.Init(start_stance, steps, 0.3, 0.2, true, true);

  auto phases = motion_structure.GetPhases();

  for (auto p : phases)
    std::cout << p << "\n\n";

//  // initially, 4 legs should be in contact, then 3 after swinging the first
//  EXPECT_EQ(6, phases.size());
//  EXPECT_EQ(4, phases.at(0).contacts_.size());
//  EXPECT_EQ(3, phases.at(1).contacts_.size());
//
//  // in the last four leg support phase, all steps should have been executed once
//  EXPECT_EQ(4, phases.back().contacts_.size());
//  EXPECT_EQ(0, phases.back().contacts_.at(0).id);
//  EXPECT_EQ(1, phases.back().contacts_.at(1).id);
//  EXPECT_EQ(2, phases.back().contacts_.at(2).id);
//  EXPECT_EQ(3, phases.back().contacts_.at(3).id);
}

} /* namespace zmp */
} /* namespace xpp */
