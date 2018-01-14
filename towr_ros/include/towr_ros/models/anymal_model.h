/*
 * anymal_model.h
 *
 *  Created on: Jan 14, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_ANYMAL_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_ANYMAL_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/centroidal_model.h>

namespace towr {

class AnymalKinematicModel : public KinematicModel {
public:
  AnymalKinematicModel ();
};


class AnymalDynamicModel : public CentroidalModel {
public:
  AnymalDynamicModel()
  : CentroidalModel(36.5,
                    1.11117, 2.20775, 2.02077, 0.00943193, 0.0101473, 0.00124553,
                    4) {}
};

} // namespace towr


#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_ANYMAL_MODEL_H_ */
