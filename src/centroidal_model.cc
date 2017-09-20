/**
 @file    centroidal_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#include <xpp/models/centroidal_model.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/cartesian_declarations.h>
#include <xpp/composite.h>
#include <xpp/endeffectors.h>
#include <xpp/state.h>

#include <xpp/quadruped_gait_generator.h>

namespace xpp {
namespace opt {

CentroidalModel::CentroidalModel (double mass, const Eigen::Matrix3d& inertia,
                                  int ee_count)
    :DynamicModel(mass)
{
  SetCurrent(ComPos::Zero(), EELoad(ee_count), EEPos(ee_count));
  I_inv_ = inertia.inverse().sparseView();
}

CentroidalModel::~CentroidalModel ()
{
  // TODO Auto-generated destructor stub
}

CentroidalModel::BaseAcc
CentroidalModel::GetBaseAcceleration () const
{
  Vector3d f_lin, ang; f_lin.setZero(); ang.setZero();

  for (auto ee : ee_pos_.GetEEsOrdered()) {
    Vector3d f = ee_force_.At(ee);
    ang += f.cross(com_pos_-ee_pos_.At(ee));
    f_lin += f;
  }

  // moved gravity to bounds, as this is constant and would mess up SNOPT
  // static const Vector3d fg_W(0.0, 0.0, -m_*kGravity);
  // f_lin += fg_W;

  BaseAcc acc;
  acc.segment(AX, kDim3d) = I_inv_*ang;
  acc.segment(LX, kDim3d) = 1./m_ *f_lin;

  return acc;
}

// just a helper function
static Jacobian
BuildCrossProductMatrix(const Vector3d& in)
{
  Jacobian out(3,3);

  out.coeffRef(0,1) = -in(2); out.coeffRef(0,2) =  in(1);
  out.coeffRef(1,0) =  in(2); out.coeffRef(1,2) = -in(0);
  out.coeffRef(2,0) = -in(1); out.coeffRef(2,1) =  in(0);

  return out;
}

Jacobian
CentroidalModel::GetJacobianOfAccWrtBaseLin (const Jacobian& jac_pos_base_lin) const
{
  // build the com jacobian
  int n = jac_pos_base_lin.cols();

  Jacobian jac_ang(kDim3d, n);
  for (const Vector3d& f : ee_force_.ToImpl())
    jac_ang += BuildCrossProductMatrix(f)*jac_pos_base_lin;

  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;

  // linear acceleration does not depend on base
  return jac;
}

Jacobian
CentroidalModel::GetJacobianOfAccWrtBaseAng (const Jacobian& jac_pos_base_ang) const
{
  // the 6D base acceleration does not depend on base orientation
  return Jacobian(kDim6d, jac_pos_base_ang.cols());
}

Jacobian
CentroidalModel::GetJacobianofAccWrtForce (const Jacobian& ee_force_jac,
                                           EndeffectorID ee) const
{
  Vector3d r = com_pos_-ee_pos_.At(ee);
  Jacobian jac_ang = -BuildCrossProductMatrix(r)*ee_force_jac;

  int n = ee_force_jac.cols();
  Jacobian jac(kDim6d, n);
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;
  jac.middleRows(LX, kDim3d) = 1./m_*ee_force_jac;

  return jac;
}

Jacobian
CentroidalModel::GetJacobianofAccWrtEEPos (const Jacobian& jac_ee_pos,
                                           EndeffectorID ee) const
{
  Vector3d f = ee_force_.At(ee);
  Jacobian jac_ang = BuildCrossProductMatrix(f)*(-jac_ee_pos);

  Jacobian jac(kDim6d, jac_ang.cols());
  jac.middleRows(AX, kDim3d) = I_inv_*jac_ang;
  // linear acceleration does not depend on endeffector position.
  return jac;
}


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

// specific models, both kinematic AND dynamic
MonopedModel::MonopedModel () : RobotModel(1)
{
  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
  dynamic_model_ = std::make_shared<CentroidalModel>(20,I,1);
  dynamic_model_->normal_force_max_ = 800;


  map_id_to_ee_["E0"] = E0;
  kinematic_model_->nominal_stance_.At(E0) = Vector3d( 0.0, 0.0, -0.58);
  kinematic_model_->max_dev_from_nominal_ << 0.25, 0.15, 0.2;

  double f   = 0.2;
  double fh  = 0.2;
  double c   = 0.2;
  contact_timings_.at(E0) = {c, f, c, f, c, f, c, fh, c, 0.4, c,
                            f, c, f, c, fh, c, f, c, f, c, f, c};
}

BipedModel::BipedModel () : RobotModel(2)
{
  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
  dynamic_model_ = std::make_shared<CentroidalModel>(20,I,2);
  dynamic_model_->normal_force_max_ = 400;

  using namespace xpp::biped;
  map_id_to_ee_ = biped::kMapIDToEE;

  const double z_nominal_b = -0.60;
  const double y_nominal_b =  0.20;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(L)) << 0.0,  y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(R)) << 0.0, -y_nominal_b, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_  << 0.25, 0.15, 0.18;



//  quad::QuadrupedGaitGenerator gait_gen;
//  gait_gen.SetGaits({quad::Trot, quad::TrotFly, quad::Bound, quad::Pace});
//  ContactTimings quad_timings = gait_gen.GetContactSchedule();
//  contact_timings_.at(kMapIDToEE.at(L)) = quad_timings.at(quad::kMapIDToEE.at(quad::LF));
//  contact_timings_.at(kMapIDToEE.at(R)) = quad_timings.at(quad::kMapIDToEE.at(quad::RF));

  double f  = 0.5;
  double c  = 0.5;
  double offset = c;
  contact_timings_.at(kMapIDToEE.at(L)) = {c+offset,f,c,f,c,f,c,f,c,f,c};
  contact_timings_.at(kMapIDToEE.at(R)) = {       c,f,c,f,c,f,c,f,c,f, c+offset};
}

HyqModel::HyqModel () : RobotModel(4)
{
  Eigen::Matrix3d I = BuildInertiaTensor( 1.209488,5.5837,6.056973,0.00571,-0.190812,-0.012668);
  dynamic_model_ = std::make_shared<CentroidalModel>(80,I,4);
  dynamic_model_->normal_force_max_ = 10000;

  using namespace xpp::quad;
  map_id_to_ee_ = quad::kMapIDToEE;

  const double x_nominal_b = 0.28;
  const double y_nominal_b = 0.28;
  const double z_nominal_b = -0.58;

//  auto kMapQuadToOpt = Reverse(quad::kMapOptToQuad);
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(LF)) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(RF)) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(LH)) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(RH)) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_ << 0.2, 0.15, 0.10;
};

void
HyqModel::SetInitialGait (int gait_id)
{
  using namespace xpp::quad;
  QuadrupedGaitGenerator gait_gen;
//  gait_gen.SetGaits({static_cast<QuadrupedGaits>(gait_id)});
  gait_gen.SetGaits({Stand, Walk2, Run2, Hop1, Run3, Stand});
  contact_timings_ = gait_gen.GetContactSchedule();
}

// from pkg anymal_description/urdf/base/anymal_base_2_parameters
AnymalModel::AnymalModel () : RobotModel(4)
{
  Eigen::Matrix3d I = BuildInertiaTensor( 0.268388530623900,
                                          0.884235660795284,
                                          0.829158678306482,
                                          0.000775392455422,
                                          -0.015184853445095,
                                          -0.000989297489507);
  dynamic_model_ = std::make_shared<CentroidalModel>(18.29 + 4*2.0,I,4);
  dynamic_model_->normal_force_max_ = 500; // spring_clean_ halved the max force



  using namespace xpp::quad;
  map_id_to_ee_ = quad::kMapIDToEE;

  const double x_nominal_b = 0.33;  // wrt to hip 5cm
  const double y_nominal_b = 0.15; // wrt to hip -3cm
  const double z_nominal_b = -0.47;

  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(LF)) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(RF)) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(LH)) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(RH)) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

  //spring_clean_ reduced endeffector range of motion
//  kinematic_model_->max_dev_from_nominal_ << 0.18, 0.08, 0.07; // for motions on real ANYmal
  kinematic_model_->max_dev_from_nominal_ << 0.23, 0.13, 0.09; // spring_clean_ reduce y range


};

void
AnymalModel::SetInitialGait (int gait_id)
{
  using namespace xpp::quad;
  QuadrupedGaitGenerator gait_gen;
//  gait_gen.SetGaits({static_cast<QuadrupedGaits>(gait_id)});
  gait_gen.SetGaits({Stand, Walk1, Stand});
  contact_timings_ = gait_gen.GetContactSchedule();

//    double f = 0.4; // [s] t_free
//    double c = 0.4; // [s] t_contact
//    double t_offset = f;
//    contact_timings_.at(kMapIDToEE.at(LH)) = {t_offset + c, f, c, f, c, f, c, f, c, f, c           };
//    contact_timings_.at(kMapIDToEE.at(LF)) = {           c, f, c, f, c, f, c, f, c, f, c + t_offset};
//    contact_timings_.at(kMapIDToEE.at(RH)) = {           c, f, c, f, c, f, c, f, c, f, c + t_offset};
//    contact_timings_.at(kMapIDToEE.at(RF)) = {t_offset + c, f, c, f, c, f, c, f, c, f, c           };
}


// from pkg xpp_urdfs/quadrotor_description/urdf/base/quadrotor.urdf
QuadrotorCentroidalModel::QuadrotorCentroidalModel () : RobotModel(4)
{
  Eigen::Matrix3d I = BuildInertiaTensor( 0.0023, 0.0023, 0.004, 0.0, 0.0, 0.0);
  dynamic_model_ = std::make_shared<CentroidalModel>(0.5, I, 4);
  dynamic_model_->normal_force_max_ = 100;

  using namespace xpp::quad_rotor;
  map_id_to_ee_ = quad_rotor::kMapIDToEE;

  const double x_nominal_b = 0.22;
  const double y_nominal_b = 0.22;
  const double z_nominal_b = 0;

//  auto kMapQuadToOpt = Reverse(kMapOptToRotor);
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(L)) <<  0,   y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(R)) <<  0,  -y_nominal_b, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(F)) <<  x_nominal_b,   0, z_nominal_b;
  kinematic_model_->nominal_stance_.At(map_id_to_ee_.at(H)) << -x_nominal_b,   0, z_nominal_b;

  kinematic_model_->max_dev_from_nominal_ << 0.0, 0.0, 0.0;
}



} /* namespace opt */
} /* namespace xpp */


