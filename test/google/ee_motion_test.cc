/**
 @file    sparse_matrix_test.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 1, 2016
 @brief   Brief description
 */

#include <xpp/opt/variables/ee_motion.h>
#include <xpp/opt/variables/endeffectors_motion.h>
#include <xpp/opt/quadruped_motion_parameters.h>

#include <gtest/gtest.h>
#include <iomanip>
#include <memory>

namespace xpp {
namespace opt {


TEST(EEMotionTest, Params)
{
  double lift_height = 0.03;
  EEMotion motion;
  motion.SetInitialPos(Vector3d(0.0, 0.0, 0.0), EndeffectorID::E0);
  motion.AddPhase(0.1, 0.0, true);
  motion.AddPhase(0.3, lift_height, false);//, Vector3d(0.5, 0.0, 0.4));
  motion.AddPhase(0.3, 0.0, true);
  motion.AddPhase(0.6, lift_height, false);//, Vector3d(0.3, 0.0, 0.0));

  std::cout << "x: " << motion.GetValues().transpose() << std::endl;
  std::cout << "n: " << motion.GetRows() << std::endl;


  VectorXd x(6); x << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  motion.SetValues(x);
  std::cout << "x_new: " << motion.GetValues().transpose() << std::endl;
  std::cout << "n_new: " << motion.GetRows() << std::endl;

  double t = 0.0;
  while (t < 1.3) {
    std::cout << std::setprecision(2) << std::fixed;
    std::cout << motion.GetState(t).p.transpose() << std::endl;
//    std::cout << motion.IsInContact(t) << std::endl;
    t += 0.1;
  }
}

TEST(EEsMotionTest, GetState)
{

  EndeffectorsPos start_stance(4);
  start_stance.At(E0) = Vector3d(+0.359692, +0.327653, 0.0);
  start_stance.At(E1) = Vector3d(+0.359694, -0.327644, 0.0);
  start_stance.At(E2) = Vector3d(-0.358797, +0.327698, 0.0);
  start_stance.At(E3) = Vector3d(-0.358802, -0.327695, 0.0);


//  Trot params;
//  auto ee_motion = std::make_shared<EndeffectorsMotion>(start_stance);
//  ee_motion->SetInitialPos(start_stance);
//  ee_motion->SetPhaseSequence(params.GetOneCycle());


//  Eigen::VectorXd x = ee_motion_->GetOptimizationParameters();
//  x.fill(1.0);
//  ee_motion_->SetOptimizationParameters(x);
//
////  auto free_contacts = ee_motion_->GetFreeContacts(0.0);
//
//  std::cout << ee_motion_->GetOptimizationParameters().transpose() << std::endl;


//  auto prev_contact_state = ee_motion->GetContactState(1.0);

//  std::cout << std::setprecision(2) << std::fixed;
//  double t = 0.0;
//  while (t <= ee_motion->GetTotalTime()) {
//
//
//    std::cout << "t: " << t << std::endl;
//    std::cout << ee_motion->GetJacobianWrtOptParams(t, E0, d2::X).toDense() << "\n";
//    std::cout << ee_motion->GetJacobianWrtOptParams(t, E0, d2::Y).toDense() << "\n";
////    std::cout << ee_motion->GetJacobianWrtOptParams(t, E2, d2::X).toDense() << "\n";
//
//
////    auto current_contact_state = ee_motion->GetContactState(t);
////    if (current_contact_state != prev_contact_state) {
////      prev_contact_state = current_contact_state;
////      std::cout << "t: " << t << std::endl;
////
////      for (auto c : ee_motion->GetContacts(t)) {
////        std::cout << c << std::endl;
////      }
////    }
//
//
//    std::cout << std::endl;
//    t += 0.1;
//  }
}


} // namespace opt
} // namespace xpp
