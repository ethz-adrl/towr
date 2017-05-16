/**
 @file    endeffector_load.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 16, 2017
 @brief   Brief description
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <xpp/endeffectors.h>

#include <xpp/bound.h>
#include <xpp/opt/constraints/composite.h>

#include "contact_schedule.h"
#include "ee_force.h"

namespace xpp {
namespace opt {


class EndeffectorsForce : public Composite {
public:
  using ComponentPtr = std::shared_ptr<EEForce>;
  using ComponentVec = std::vector<ComponentPtr>;
  using LoadParams   = Endeffectors<double>;

  EndeffectorsForce(double dt, const ContactSchedule&);
  virtual ~EndeffectorsForce();

  LoadParams GetLoadValues(double t) const;
  JacobianRow GetJacobian (double t, EndeffectorID ee, Coords3D dim) const;

private:
  ComponentVec ee_forces_; // derived class pointer to access ee specific functions
  std::vector<EndeffectorID> ee_ordered_;
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_ENDEFFECTOR_LOAD_H_ */
