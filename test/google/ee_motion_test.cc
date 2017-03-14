/**
 @file    sparse_matrix_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/opt/ee_motion.h>
#include <gtest/gtest.h>
#include <iomanip>

namespace xpp {
namespace opt {


TEST(EEMotionTest, GetState) {

  EEMotion motion;
  motion.SetInitialPos(Vector3d(0.0, 0.0, 0.0));
  motion.AddStancePhase(0.1);
  motion.AddSwingPhase(0.3, Vector3d(0.5, 0.0, 0.4));
  motion.AddStancePhase(0.3);
  motion.AddSwingPhase(0.6, Vector3d(0.3, 0.0, 0.0));

  double t = 0.0;
  while (t < 1.3) {
    std::cout << std::setprecision(2) << std::fixed;
    std::cout << motion.GetState(t) << std::endl;
    std::cout << motion.IsInContact(t) << std::endl;
    t += 0.01;
  }
}

} // namespace opt
} // namespace xpp
