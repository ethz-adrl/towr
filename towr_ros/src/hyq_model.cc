/*
 * hyq_model.cc
 *
 *  Created on: Jan 14, 2018
 *      Author: winklera
 */

#include <towr_ros/models/hyq_model.h>

#include <xpp_states/endeffector_mappings.h>

namespace towr {

HyqKinematicModel::HyqKinematicModel () : KinematicModel(4)
{
  const double x_nominal_b = 0.31;
  const double y_nominal_b = 0.29;
  const double z_nominal_b = -0.58;

  nominal_stance_.at(xpp::quad::LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::quad::RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

  max_dev_from_nominal_ << 0.15, 0.06, 0.1;
}

} /* namespace towr */
