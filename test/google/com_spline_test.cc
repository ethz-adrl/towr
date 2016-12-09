/**
 @file    com_spline_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/com_spline.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/motion_structure.h>
#include <xpp/hyq/foothold.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

using namespace Eigen;

TEST(ComSplineTest, Phase) {

  // create the fixed motion structure
  MotionStructure motion_structure;
  motion_structure.Init({}, {hyq::LH, hyq::LF}, 0.7, 0.4, true, true, 0.1);

  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetPhases(),
                                                   Vector2d(0.0,0.0), // pos
                                                   Vector2d(0.0,0.2) // vel
                                                   );

  std::cout << "phases:\n";
  for (auto phase : motion_structure.GetPhases()) {
    std::cout << phase << std::endl;
  }

  auto com_spline = std::dynamic_pointer_cast<ComSpline>(com_motion);
}

} /* namespace zmp */
} /* namespace xpp */
