/**
 @file    kinematic_model.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Sep 19, 2017
 @brief   Brief description
 */

#include <towr_ros/models/anymal_model.h>

#include <xpp_states/endeffector_mappings.h>

namespace towr {


AnymalKinematicModel::AnymalKinematicModel () : KinematicModel(4)
{
  const double x_nominal_b = 0.33; // wrt to hip 5cm
  const double y_nominal_b = 0.19; // wrt to hip -3cm
  const double z_nominal_b = -0.46;

  nominal_stance_.at(xpp::quad::LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

  max_dev_from_nominal_ << 0.15, 0.1, 0.10;
}

} // namespace towr


