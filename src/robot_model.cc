/**
 @file    robot_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include <xpp/models/robot_model.h>

#include <map>
#include <memory>
#include <Eigen/Dense>

#include <xpp/models/monoped_gait_generator.h>
#include <xpp/models/biped_gait_generator.h>
#include <xpp/models/quadruped_gait_generator.h>

#include <xpp/endeffectors.h>
#include <xpp/models/centroidal_model.h>
#include <xpp/state.h>

namespace xpp {
namespace opt {

static Eigen::Matrix3d BuildInertiaTensor(
        double Ixx, double Iyy, double Izz,
        double Ixy, double Ixz, double Iyz)
{
  Eigen::Matrix3d I;
  I <<  Ixx, -Ixy, -Ixz,
       -Ixy,  Iyy, -Iyz,
       -Ixz, -Iyz,  Izz;
  return I;
}

void
RobotModel::MakeMonopedModel ()
{
  int n_ee = 1;

  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
  dynamic_model_ = std::make_shared<CentroidalModel>(20, I, n_ee);
  dynamic_model_->SetForceLimit(800);

  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  kinematic_model_->nominal_stance_.At(E0) = Vector3d( 0.0, 0.0, -0.58);
  kinematic_model_->max_dev_from_nominal_ << 0.25, 0.15, 0.2;

  gait_generator_ = std::make_shared<MonopedGaitGenerator>();
}

void
RobotModel::MakeBipedModel ()
{
  using namespace xpp::biped;
  int n_ee = kMapIDToEE.size();

  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
  dynamic_model_ = std::make_shared<CentroidalModel>(20, I, n_ee);
  dynamic_model_->SetForceLimit(400);


  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double z_nominal_b = -0.60;
  const double y_nominal_b =  0.20;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(L)) << 0.0,  y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(R)) << 0.0, -y_nominal_b, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_  << 0.25, 0.15, 0.18;

  gait_generator_ = std::make_shared<BipedGaitGenerator>();
}

void
RobotModel::MakeHyqModel ()
{
  using namespace xpp::quad;
  int n_ee = kMapIDToEE.size();

//  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);

  // for hyq standing in nominal stance, legs taken into account
  Eigen::Matrix3d I = BuildInertiaTensor( 4.26, 8.97, 9.88, -0.0063, 0.193, 0.0126);



  dynamic_model_ = std::make_shared<CentroidalModel>(83, I, n_ee);
  dynamic_model_->SetForceLimit(10000);



  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double x_nominal_b = 0.31;
  const double y_nominal_b = 0.29;
  const double z_nominal_b = -0.56;

  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(LF)) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(RF)) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(LH)) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(RH)) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_ << 0.15, 0.07, 0.07;

  gait_generator_ = std::make_shared<QuadrupedGaitGenerator>();
}

void
RobotModel::MakeAnymalModel ()
{
  using namespace xpp::quad;
  int n_ee = kMapIDToEE.size();

  Eigen::Matrix3d I = BuildInertiaTensor( 0.268388530623900,
                                          0.884235660795284,
                                          0.829158678306482,
                                          0.000775392455422,
                                          -0.015184853445095,
                                          -0.000989297489507);
  dynamic_model_ = std::make_shared<CentroidalModel>(18.29 + 4*2.0, I, n_ee);
  dynamic_model_->SetForceLimit(500);


  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double x_nominal_b = 0.33;  // wrt to hip 5cm
  const double y_nominal_b = 0.15; // wrt to hip -3cm
  const double z_nominal_b = -0.47;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(LF)) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(RF)) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(LH)) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(RH)) << -x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->max_dev_from_nominal_ << 0.23, 0.13, 0.09;

  gait_generator_ = std::make_shared<QuadrupedGaitGenerator>();
}

void
RobotModel::MakeQuadrotorModel ()
{
  using namespace xpp::quad_rotor;
  int n_ee = kMapIDToEE.size();

  Eigen::Matrix3d I = BuildInertiaTensor( 0.0023, 0.0023, 0.004, 0.0, 0.0, 0.0);
  dynamic_model_ = std::make_shared<CentroidalModel>(0.5, I, n_ee);
  dynamic_model_->SetForceLimit(100);



  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double x_nominal_b = 0.22;
  const double y_nominal_b = 0.22;
  const double z_nominal_b = 0;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(L)) <<  0,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(R)) <<  0,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(F)) <<  x_nominal_b,   0, z_nominal_b;
  kinematic_model_->nominal_stance_.At(kMapIDToEE.at(H)) << -x_nominal_b,   0, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_ << 0.0, 0.0, 0.0;

  gait_generator_ = std::make_shared<QuadrupedGaitGenerator>();
}

} /* namespace opt */
} /* namespace xpp */
