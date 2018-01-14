/*
 * hyq_model.h
 *
 *  Created on: Jan 14, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/centroidal_model.h>

namespace towr {

class HyqKinematicModel : public KinematicModel {
public:
  HyqKinematicModel ();
};


class HyqDynamicModel : public CentroidalModel {
public:
  HyqDynamicModel() : CentroidalModel(83,
                      4.26, 8.97, 9.88, -0.0063, 0.193, 0.0126,
                      4) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_ */
