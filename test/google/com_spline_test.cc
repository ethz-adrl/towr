/**
 @file    com_spline_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <xpp/opt/com_spline.h>
#include <xpp/opt/motion_factory.h>
#include <xpp/opt/motion_structure.h>

#include <gtest/gtest.h>

namespace xpp {
namespace opt {

using namespace Eigen;
using EndeffectorID = utils::EndeffectorID;

TEST(ComSplineTest, MotionPhase) {

//  // create the fixed motion structure
//  MotionStructure motion_structure;
//  motion_structure.Init({}, {{EndeffectorID::E0}, {EndeffectorID::E1}}, 0.7, 0.0, 0.1);
//
//  auto com_motion = MotionFactory::CreateComMotion(motion_structure.GetTotalTime(),1,
//                                                   Vector2d(0.0,0.0), // pos
//                                                   Vector2d(0.0,0.2) // vel
//                                                   );
//
//  std::cout << "phases:\n";
//  for (auto phase : motion_structure.GetPhases()) {
//    std::cout << phase << std::endl;
//  }
//
//  auto com_spline = std::dynamic_pointer_cast<ComSpline>(com_motion);
}

} /* namespace zmp */
} /* namespace xpp */
