/**
 @file    dynamic_constraint_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Dec 5, 2016
 @brief   Brief description
 */

#include <xpp/endeffectors.h>
#include <xpp/opt/ee_phase_motion.h>
#include <gtest/gtest.h>

namespace xpp {
namespace opt {


TEST(EndeffectorPhaseMotion, Derivatives)
{
  EEPhaseMotion motion;
  double T = 1.0;
  double h = 0.03;
  Vector3d start(0.0, 0.0, 0.0);
  Vector3d end(0.5, 0.3, 0.0);
  motion.Init(T,h,start,end);


  double t = 0.0;
  while (t < T) {

    std::cout << "t: " << t << std::endl;
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::X, t, Polynomial::Start) << "\t";
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::Y, t, Polynomial::Start) << "\t";
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::X, t, Polynomial::Goal)  << "\t";
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::Y, t, Polynomial::Goal)  << "\t";
    std::cout << std::endl;

    t += 0.1;
  }
}


TEST(EndeffectorMotion, Derivatives)
{
  EEPhaseMotion motion;
  double T = 1.0;
  double h = 0.03;
  Vector3d start(0.0, 0.0, 0.0);
  Vector3d end(0.5, 0.3, 0.0);
  motion.Init(T,h,start,end);


  double t = 0.0;
  while (t < T) {

    std::cout << "t: " << t << std::endl;
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::X, t, Polynomial::Start) << "\t";
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::Y, t, Polynomial::Start) << "\t";
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::X, t, Polynomial::Goal)  << "\t";
    std::cout << motion.GetDerivativeOfPosWrtContactsXY(d2::Y, t, Polynomial::Goal)  << "\t";
    std::cout << std::endl;

    t += 0.1;
  }
}




} /* namespace opt */
} /* namespace xpp */
