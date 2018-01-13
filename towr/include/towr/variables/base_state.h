/*
 * base_state.h
 *
 *  Created on: Jan 12, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_STATE_H_
#define TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_STATE_H_


#include <Eigen/Dense>

namespace towr {

// smell move to own class
struct BaseState {
  // linear
  Eigen::Vector3d pos_xyz_;
  Eigen::Vector3d vel_xyz_;
  // angular
  Eigen::Vector3d pos_rpy_;
  Eigen::Vector3d vel_rpy_;
};

using Vector3d = Eigen::Vector3d;
using NewEEPos = std::vector<Vector3d>;


} // namespace towr


#endif /* TOWR_TOWR_INCLUDE_TOWR_VARIABLES_BASE_STATE_H_ */
