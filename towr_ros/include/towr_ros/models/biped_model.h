/*
 * biped_model.h
 *
 *  Created on: Jan 14, 2018
 *      Author: winklera
 */

#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_BIPED_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_BIPED_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/centroidal_model.h>

namespace towr {

class BipedKinematicModel : public KinematicModel {
public:
  BipedKinematicModel ();
};


class BipedDynamicModel : public CentroidalModel {
public:
  BipedDynamicModel()
  : CentroidalModel(20,
                    1.209,5.583,6.056,0.005,-0.190,-0.012,
                    2) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_BIPED_MODEL_H_ */
