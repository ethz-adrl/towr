/**
 @file    centroidal_model.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_
#define XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_

#include "dynamic_model.h"

namespace xpp {
namespace opt {

/**
 * @brief Centroidal Dynamics for the 6-DoF Base to model the system.
 */
// zmp_ !!! implement this
class CentroidalModel : public DynamicModel {
public:
  CentroidalModel ();
  virtual ~CentroidalModel ();
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_CENTROIDAL_MODEL_H_ */
