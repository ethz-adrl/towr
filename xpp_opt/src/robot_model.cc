/**
 @file    robot_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include <xpp_opt/models/robot_model.h>

#include <Eigen/Dense>
#include <map>
#include <memory>

#include <xpp_opt/models/biped_gait_generator.h>
#include <xpp_opt/models/centroidal_model.h>
#include <xpp_opt/models/monoped_gait_generator.h>
#include <xpp_opt/models/quadruped_gait_generator.h>
#include <xpp_states/endeffectors.h>
#include <xpp_states/state.h>
#include <xpp_states/endeffector_mappings.h>


namespace xpp {

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
  kinematic_model_->nominal_stance_.at(0) = Vector3d( 0.0, 0.0, -0.58);
  kinematic_model_->max_dev_from_nominal_ << 0.25, 0.15, 0.2;

  gait_generator_ = std::make_shared<MonopedGaitGenerator>();
}

void
RobotModel::MakeBipedModel ()
{
  using namespace biped; // for L, R definitions
  int n_ee = 2;

  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
  dynamic_model_ = std::make_shared<CentroidalModel>(20, I, n_ee);
  dynamic_model_->SetForceLimit(5000);


  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double z_nominal_b = -0.65;
  const double y_nominal_b =  0.20;
  kinematic_model_->nominal_stance_.at(L) << 0.0,  y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(R) << 0.0, -y_nominal_b, z_nominal_b;

//  kinematic_model_->max_dev_from_nominal_  << 0.25, 0.15, 0.10;
  kinematic_model_->max_dev_from_nominal_  << 0.3, 0.15, 0.15;

  gait_generator_ = std::make_shared<BipedGaitGenerator>();
}

void
RobotModel::MakeHyqModel ()
{
  using namespace xpp::quad;
  int n_ee = 4;

//  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);

  // for hyq standing in nominal stance, legs taken into account
  Eigen::Matrix3d I = BuildInertiaTensor( 4.26, 8.97, 9.88, -0.0063, 0.193, 0.0126);



  dynamic_model_ = std::make_shared<CentroidalModel>(83, I, n_ee);
  dynamic_model_->SetForceLimit(10000);



  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double x_nominal_b = 0.31;
  const double y_nominal_b = 0.29;
  const double z_nominal_b = -0.58;
//  const double z_nominal_b = -0.60;

  kinematic_model_->nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

//  kinematic_model_->max_dev_from_nominal_ << 0.15, 0.07, 0.05;

  kinematic_model_->max_dev_from_nominal_ << 0.15, 0.06, 0.1;

  gait_generator_ = std::make_shared<QuadrupedGaitGenerator>();
}

void
RobotModel::MakeAnymalModel ()
{
  using namespace xpp::quad;
  int n_ee = 4;

//  Eigen::Matrix3d I = BuildInertiaTensor( 0.268388530623900,
//                                          0.884235660795284,
//                                          0.829158678306482,
//                                          0.000775392455422,
//                                          -0.015184853445095,
//                                          -0.000989297489507);

  // adapted from anymal standing with all legs on ground
  Eigen::Matrix3d I = BuildInertiaTensor( 1.11117,
		                          2.20775,
		                          2.02077,
		                          0.00943193,
		                          0.0101473,
		                          0.00124553);



  dynamic_model_ = std::make_shared<CentroidalModel>(36.5, I, n_ee);
  dynamic_model_->SetForceLimit(3000);


  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double x_nominal_b = 0.33; // wrt to hip 5cm
  const double y_nominal_b = 0.19; // wrt to hip -3cm
  const double z_nominal_b = -0.46;
  kinematic_model_->nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->max_dev_from_nominal_ << 0.15, 0.15, 0.13;
//  kinematic_model_->max_dev_from_nominal_ << 0.15, 0.06, 0.10; // for real robot tests

  gait_generator_ = std::make_shared<QuadrupedGaitGenerator>();
}

void
RobotModel::SetAnymalInitialState (State3dEuler& base, EndeffectorsPos& feet)
{
  using namespace xpp::quad;

  base.lin.p_ << 0.0, 0.0, 0.46;  // for real robot 0.46, in simulation probably higher.
  base.ang.p_ << 0.0, 0.0, 0.0;    // euler (roll, pitch, yaw)

  int n_ee = 4;
  double z_start = -0.02; // for real robot approx -0.02, in simulation +0.02.
  feet.SetCount(n_ee);
  feet.at(LF) <<  0.34,  0.19, z_start;
  feet.at(RF) <<  0.34, -0.19, z_start;
  feet.at(LH) << -0.34,  0.19, z_start;
  feet.at(RH) << -0.34, -0.19, z_start;
}

void
RobotModel::SetInitialState(State3dEuler& base, EndeffectorsPos& feet) const
{
  double z_start = kinematic_model_->nominal_stance_.at(0).z();

  base.lin.p_ << 0.0, 0.0, -z_start;  // for real robot 0.46, in simulation probably higher.
  base.ang.p_ << 0.0, 0.0, 0.0;      // euler (roll, pitch, yaw)

  feet = kinematic_model_->nominal_stance_;
  for (int i=0; i<feet.GetEECount(); ++i)
    feet.at(i).z() = 0.0; // ground level
}

void
RobotModel::MakeQuadrotorModel ()
{
  using namespace xpp::quad_rotor;
  int n_ee = 4;

  Eigen::Matrix3d I = BuildInertiaTensor( 0.0023, 0.0023, 0.004, 0.0, 0.0, 0.0);
  dynamic_model_ = std::make_shared<CentroidalModel>(0.5, I, n_ee);
  dynamic_model_->SetForceLimit(100);



  kinematic_model_ = std::make_shared<KinematicModel>(n_ee);
  const double x_nominal_b = 0.22;
  const double y_nominal_b = 0.22;
  const double z_nominal_b = 0;
  kinematic_model_->nominal_stance_.at(L) <<  0,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(R) <<  0,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.at(F) <<  x_nominal_b,   0, z_nominal_b;
  kinematic_model_->nominal_stance_.at(H) << -x_nominal_b,   0, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_ << 0.0, 0.0, 0.0;

  gait_generator_ = std::make_shared<QuadrupedGaitGenerator>();
}

} /* namespace xpp */

