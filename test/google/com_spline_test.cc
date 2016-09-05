/**
 @file    com_spline_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 23, 2016
 @brief   Brief description
 */

#include <gtest/gtest.h>
#include <xpp/zmp/motion_factory.h>
#include <xpp/zmp/com_spline.h>

namespace xpp {
namespace zmp {

using namespace Eigen;

TEST(ComSplineTest, PhaseInfo) {

  auto com_motion = MotionFactory::CreateComMotion(Vector2d(0.0,0.0), // pos
                                                   Vector2d(0.0,0.2), // vel
                                                   2,
                                                   SplineTimes(),
                                                   true);

  std::cout << "phases:\n";
  for (auto phase : com_motion->GetPhases()) {
    std::cout << phase << std::endl;
  }

  auto com_spline = dynamic_cast<ComSpline*>(com_motion.get());

  std::cout << "polynomials:\n";
  for (auto poly : com_spline->GetPolynomials()) {
    std::cout << poly << std::endl;
  }

}

} /* namespace zmp */
} /* namespace xpp */
