/*
 * monoped_model.cc
 *
 *  Created on: Jan 14, 2018
 *      Author: winklera
 */

#include <towr_ros/models/monoped_model.h>

namespace towr {

MonopedKinematicModel::MonopedKinematicModel () : KinematicModel(1)
{
  nominal_stance_.at(0) = Eigen::Vector3d( 0.0, 0.0, -0.58);
  max_dev_from_nominal_ << 0.25, 0.15, 0.2;
}

} /* namespace towr */
