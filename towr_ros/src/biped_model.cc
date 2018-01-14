/*
 * biped_model.cc
 *
 *  Created on: Jan 14, 2018
 *      Author: winklera
 */

#include <towr_ros/models/biped_model.h>

#include <xpp_states/endeffector_mappings.h>

namespace towr {

BipedKinematicModel::BipedKinematicModel () : KinematicModel(2)
{
  const double z_nominal_b = -0.65;
  const double y_nominal_b =  0.20;

  nominal_stance_.at(xpp::biped::L) << 0.0,  y_nominal_b, z_nominal_b;
  nominal_stance_.at(xpp::biped::R) << 0.0, -y_nominal_b, z_nominal_b;

  max_dev_from_nominal_  << 0.25, 0.15, 0.15;
}

} /* namespace towr */
