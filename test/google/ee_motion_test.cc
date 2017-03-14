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


  EEMotion::Timings timings;
  timings.push_back({0.1, 0.3}); // stance, swing
  timings.push_back({0.1, 0.3}); // stance, swing

  EEMotion::Contacts contacts;
  contacts.push_back(Vector3d(0.0, 0.0, 0.0));
  contacts.push_back(Vector3d(0.5, 0.0, 0.0));
  contacts.push_back(Vector3d(0.6, 0.0, 0.0));

  EEMotion motion;
  motion.SetParameters(timings, contacts);


  double t = 0.0;
  while (t < 0.8) {
    std::cout << std::setprecision(2) << std::fixed;
    std::cout << motion.GetState(t) << std::endl;
    std::cout << motion.IsInContact(t) << std::endl;
    t += 0.01;
  }
}

} // namespace opt
} // namespace xpp
